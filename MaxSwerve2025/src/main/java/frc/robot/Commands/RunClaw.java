// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunClaw extends Command {
  /** Creates a new RunClaw. */
  private final ClawevatorSubsystem clawSubsystem;
  private final DoubleSupplier axis;
  public RunClaw(ClawevatorSubsystem clawSubsystem, DoubleSupplier axis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.clawSubsystem = clawSubsystem;
    this.axis = axis;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = axis.getAsDouble();
    if(clawSubsystem.algaeCheck() && axis.getAsDouble() > 0)
      speed = 0;
    clawSubsystem.runClaw(speed, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    clawSubsystem.runClaw(0, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
