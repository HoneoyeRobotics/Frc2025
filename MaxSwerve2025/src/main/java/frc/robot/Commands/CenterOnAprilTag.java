// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterOnAprilTag extends Command {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final boolean goLeft;

  /** Creates a new CenterOnAprilTag. */
  public CenterOnAprilTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, boolean goLeft) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.goLeft = goLeft;
    addRequirements(driveSubsystem);
  }

  private PIDController pid = new PIDController(0.1, 0, 0);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(0.1, 0.001, 0);
    visionSubsystem.setDriveServo(RobotConstants.DriveServoBottom);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double speed = 0.20; //30% speed;
    // get the current x value from limelight
    double maxSpeed = 0.2;
    double speed = maxSpeed;

    if (visionSubsystem.canSeeTag()) {
      double x = visionSubsystem.getX();

      speed = pid.calculate(x);
      if (speed > maxSpeed)
        speed = maxSpeed;
      else if (speed < -maxSpeed)
        speed = -maxSpeed;

    } else {
      if(goLeft == false)
        speed *= -1;

    }

    // if to the left, move left.
    // if(x > 0)
    // speed = speed * -1;

    driveSubsystem.drive(0, speed, 0, false, false);
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
    return visionSubsystem.canSeeTag() && visionSubsystem.getOnTarget();
  }
}
