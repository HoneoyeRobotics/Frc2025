// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ForwardToAprilTag extends Command {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final double xOffset;
  private final double yOffset;
  private final Boolean TagBelow;
  private final Boolean IgnoreX;

  /** Creates a new CenterOnAprilTag. */
  public ForwardToAprilTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Boolean TagBelow,
      boolean IgnoreX,
      double xOffset,
      double yOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.TagBelow = TagBelow;
    this.xOffset = xOffset;
    this.yOffset = yOffset;
    this.IgnoreX = IgnoreX;
    addRequirements(driveSubsystem);
  }

  private PIDController turnpid = new PIDController(0.1, 0, 0);

  private PIDController fwdpid = new PIDController(0.1, 0, 0);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSubsystem.setDriveServo(RobotConstants.DriveServoBottom);

    fwdpid.setP(Preferences.getDouble("fwdp", 0.1));
    fwdpid.setI(Preferences.getDouble("fwdi", 0.0));
    fwdpid.setD(Preferences.getDouble("fwdd", 0.0));
    fwdpid.setIntegratorRange(-1, 1);

    turnpid.setP(Preferences.getDouble("turnp", 0.1));
    turnpid.setI(Preferences.getDouble("turni", 0.0));
    turnpid.setD(Preferences.getDouble("turnd", 0.0));
    turnpid.setIntegratorRange(-1, 1);
    turnpid.setSetpoint(xOffset);
    fwdpid.setSetpoint(yOffset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double speed = 0.20; //30% speed;
    // get the current x value from limelight
    double maxSpeed = 0.2;
    double xSpeed = 0;

    if (visionSubsystem.canSeeTag()) {
      double x = visionSubsystem.getX();

      xSpeed = turnpid.calculate(x);
      if (xSpeed > maxSpeed)
        xSpeed = maxSpeed;
      else if (xSpeed < -maxSpeed)
        xSpeed = -maxSpeed;

    }
    if (IgnoreX)
      xSpeed = 0;
    double maxySpeed = 0.2;

    double ySpeed = maxySpeed;
    if (visionSubsystem.canSeeTag()) {
      double y = visionSubsystem.getY();

      ySpeed = fwdpid.calculate(y);
      if (ySpeed > maxSpeed)
        ySpeed = maxSpeed;
      else if (ySpeed < -maxSpeed)
        ySpeed = -maxSpeed;
      if (TagBelow)
        ySpeed *= -1;

    }

    if(xSpeed > 0.04 || xSpeed < -0.04)
      ySpeed = 0;

    SmartDashboard.putNumber("yspeed", ySpeed);
    SmartDashboard.putNumber("xpseed", xSpeed);
    // if to the left, move left.
    // if(x > 0)
    // speed = speed * -1;


    double rotation = visionSubsystem.getRotation() * (TagBelow ? -1 : 1) * 0.01;
    driveSubsystem.drive(ySpeed, xSpeed, rotation, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false, false);

    visionSubsystem.setDriveServo(RobotConstants.DriveServoStraight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return visionSubsystem.canSeeTag() && visionSubsystem.getOnTarget();
    return false;
  }
}
