// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ClawevatorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  private final DriveSubsystem driveSubsystem;

  private final CommandXboxController driverController;
  private final VisionSubsystem visionSubsystem;
  private final ClawevatorSubsystem clawevatorSubsystem;

  /** Creates a new CenterOnAprilTag. */
  public TeleopDrive(DriveSubsystem driveSubsystem, CommandXboxController driverController,
      VisionSubsystem visionSubsystem, ClawevatorSubsystem clawevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.driverController = driverController;
    this.clawevatorSubsystem = clawevatorSubsystem;
    addRequirements(driveSubsystem);
  }

  private PIDController yPidController = new PIDController(0.03, 0.001, 0.001);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yPidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband);
    double ySpeed = -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband);
    double zRot = -MathUtil.applyDeadband(
        driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband);

    double RightxSpeed = -MathUtil.applyDeadband(driverController.getRightY(), OIConstants.kDriveDeadband);
    double RightySpeed = -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);

    boolean fieldRelative = true;

    // if the left stick is all zero, but the right stick is not, then use the robot
    // relative drive.
    if (xSpeed == 0 && ySpeed == 0 && (RightxSpeed != 0 || RightySpeed != 0)) {
      fieldRelative = false;
      xSpeed = RightxSpeed;
      ySpeed = RightySpeed;
    }

    // if you are pushing on the left stick, modify based ont he limelight.
    if (driverController.leftBumper().getAsBoolean() == true) {

      fieldRelative = false;
      xSpeed = RightxSpeed;
      ySpeed = RightySpeed;
      // can you see an april tag??

      double tag = LimelightHelpers.getFiducialID("limelight-drive");

      if (tag > 0) {

        // calculate side to side difference.
        double currentTX = LimelightHelpers.getTX("limelight-drive");
        double xoffset = 0;

        if (clawevatorSubsystem.isCoral())
          xoffset = 0.539404 * visionSubsystem.getA() + 0.832159; // coral
        else
          xoffset = (-0.911955 * visionSubsystem.getA()) - 4.56205; // algae
        yPidController.setSetpoint(xoffset);
        double strafePower = yPidController.calculate(currentTX) ;//* (clawevatorSubsystem.isCoral() ? 1 :-1);
        ySpeed = strafePower * (1 + Math.abs(ySpeed));

        // now check for how off rotationally we are
        double heading = MathUtil.inputModulus(driveSubsystem.getHeading(), -360, 360);

        double degreesOff = visionSubsystem.getRotation();
        double rotatekP = .035;
        double rotateVelocity = degreesOff * rotatekP;
        double rotatespeedModifier = Math.abs(xSpeed);
        if (rotatespeedModifier < 0.10 && rotatespeedModifier > -0.10)
          rotatespeedModifier = 0.10 * (rotatespeedModifier < 0 ? -1 : 0);
        rotateVelocity *= rotatespeedModifier * -1;
        zRot += rotateVelocity;

        // are we too close and going towards it
        if (visionSubsystem.getA() > 17 && xSpeed < 0)
          xSpeed = 0;
      }
    }

    // normal driving;
    driveSubsystem.drive(
        xSpeed,
        ySpeed,
        zRot,
        fieldRelative,
        driverController.rightBumper().getAsBoolean()

    );
  }

  public double limelight_aim_proportional() {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our
    // proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
    // rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight-drive") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= 1;

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are
  // different.
  // if your limelight and target are mounted at the same or similar heights, use
  // "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-drive") * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
