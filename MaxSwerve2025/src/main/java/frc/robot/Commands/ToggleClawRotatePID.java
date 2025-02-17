// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToggleClawRotatePID extends InstantCommand {
  /** Creates a new RunClaw. */
  private final ClawevatorSubsystem clawSubsystem;

  public ToggleClawRotatePID(ClawevatorSubsystem clawSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.clawSubsystem = clawSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    clawSubsystem.ToggleClawRotatePID();
  }

}
