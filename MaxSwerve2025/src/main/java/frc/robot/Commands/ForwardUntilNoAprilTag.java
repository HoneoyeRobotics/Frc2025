// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ForwardUntilNoAprilTag extends Command {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final double speed;

  /** Creates a new CenterOnAprilTag. */
  public ForwardUntilNoAprilTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.speed = speed;
    
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveSubsystem.drive(speed, 0, 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false, false);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return visionSubsystem.canSeeTag() && visionSubsystem.getOnTarget();
    if (visionSubsystem.canSeeTag() && visionSubsystem.getX() > 12)
      return true;

    return visionSubsystem.canSeeTag() == false;
  }
}
