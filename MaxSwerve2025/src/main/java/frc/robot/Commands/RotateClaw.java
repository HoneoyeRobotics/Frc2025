// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateClaw extends InstantCommand {
  /** Creates a new RunClaw. */
  private final ClawSubsystem clawSubsystem;
  private final double modifier;

  public RotateClaw(ClawSubsystem clawSubsystem, double modifier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.clawSubsystem = clawSubsystem;
    this.modifier = modifier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double setpoint = Preferences.getDouble("RotateMoveValue", 0.05);
    setpoint *= modifier;
    clawSubsystem.AdjustRotatePosition(setpoint);
  }

}
