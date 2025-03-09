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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  private final DriveSubsystem driveSubsystem;

  private final CommandXboxController driverController;
  private final VisionSubsystem visionSubsystem;

  /** Creates a new CenterOnAprilTag. */
  public TeleopDrive(DriveSubsystem driveSubsystem, CommandXboxController driverController, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.driverController = driverController;
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
    if (driverController.rightBumper().getAsBoolean() == true) {

      // can you see an april tag??

      double tag = LimelightHelpers.getFiducialID("limelight-drive");

      if (tag > 0) {

        // strafe
        // double strafekP = .03;
        double xoffset = -15;        
        double currentTX = LimelightHelpers.getTX("limelight-drive");

        xoffset  = (-0.911955 * visionSubsystem.getA()) - 4.56205;

        double diffTXtoTarget = currentTX - xoffset;// ex at 23, needs to be at -15, so should be 38

        
        yPidController.setSetpoint(xoffset);
        double strafePower = yPidController.calculate(currentTX) * -1;

        // double strafeVelocity = (diffTXtoTarget) * strafekP; // if 39, should be
        // 0.475 (47.5%)
        // System.out.println("tx plus offset " +(currentTX - xoffset) +"; SKP: " +
        // strafekP );
        // double speedModifier = Math.abs(xSpeed);
        // //if we are going fast, we need to speed up the number.
        // if(speedModifier > 0.10 || speedModifier < -0.10)
        // {
        // //example, we want to adjust by 0.25; but we are going fast and want to go
        // faster to the

        // speedModifier = speedModifier * 2;

        // }
        // if(speedModifier != 0 )
        // strafeVelocity *= speedModifier;

        // invert since tx is positive when the target is to the right of the crosshair

        // if we are going towards us (negative speed based on field relative) then we
        // want to use the power and they send
        ySpeed = strafePower * (1 + Math.abs(ySpeed));

        // System.out.println("strafePower: "+strafePower+"; ySpeed: " + ySpeed + ";
        // diffTXtoTarget: " + diffTXtoTarget + "; currentTX:" + currentTX);

        // zRot = rot_limelight;

        // double forward_limelight = limelight_range_proportional();
        // xSpeed = forward_limelight;
        double targetAngle = 0;
        if (tag == 6)
          targetAngle = 180;
        else if (tag == 2)
          targetAngle = -90;

        double heading = MathUtil.inputModulus(driveSubsystem.getHeading(), -360, 360);
        double targetRotation = targetAngle;
        if (heading < 0) {
          targetRotation = targetRotation - 360;
        }

        double degreesOff = 0;
        degreesOff = targetRotation - heading;



         degreesOff = visionSubsystem.getRotation();
        // if(heading < 0){
        // closest *= -1;
        // }
        // System.out.println("heading: " + heading + "; Target: " + targetRotation + ";
        // off: " + degreesOff);

        // we want to be 180, find where we are

        // if the remainder is more than half of the current setpoint, then we want to
        // go back rather than forward.

        double rotatekP = .035;
        double rotateVelocity = degreesOff * rotatekP;
        double rotatespeedModifier = Math.abs(xSpeed);
        if (rotatespeedModifier < 0.10 && rotatespeedModifier > -0.10)
          rotatespeedModifier = 0.10 * (rotatespeedModifier < 0 ? -1 : 0);

        rotateVelocity *= rotatespeedModifier;
        zRot += rotateVelocity;
      }
    }

    // normal driving;
    driveSubsystem.drive(
        xSpeed,
        ySpeed,
        zRot,
        fieldRelative,
        driverController.leftBumper().getAsBoolean()

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
