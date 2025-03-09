// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ForwardToLeftCoral extends Command {
  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final double speed;

  /** Creates a new CenterOnAprilTag. */
  public ForwardToLeftCoral(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.speed = speed;

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

    double xSpeed = speed;
    double ySpeed = 0;
    double zRot = 0;

    // calculate side to side difference.
    double currentTX = LimelightHelpers.getTX("limelight-drive");
    // double xoffset = (-0.911955 * visionSubsystem.getA()) - 4.56205; //algae
    double xoffset = 0.539404 * visionSubsystem.getA() + 0.832159; // coral
    yPidController.setSetpoint(xoffset);

    double strafePower = yPidController.calculate(currentTX);
    ySpeed = strafePower * (1 + Math.abs(ySpeed));
    System.out.println("Target Value: " + xoffset + "; position: " + currentTX + "; power: " + ySpeed);
    double degreesOff = visionSubsystem.getRotation();
    double rotatekP = .035;
    double rotateVelocity = degreesOff * rotatekP;
    double rotatespeedModifier = Math.abs(xSpeed);
    if (rotatespeedModifier < 0.10 && rotatespeedModifier > -0.10)
      rotatespeedModifier = 0.10 * (rotatespeedModifier < 0 ? -1 : 0);
    rotateVelocity *= rotatespeedModifier * -1;
    zRot += rotateVelocity;

    driveSubsystem.drive(
        xSpeed,
        ySpeed,
        zRot,
        false,
        false

    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return visionSubsystem.canSeeTag() && visionSubsystem.getOnTarget();
    if (visionSubsystem.canSeeTag() && visionSubsystem.getA() > 8)
      return true;

    return visionSubsystem.canSeeTag() == false;
  }
}
