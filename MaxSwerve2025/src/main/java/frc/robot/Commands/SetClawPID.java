// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetClawPID extends InstantCommand {

  private ClawevatorSubsystem clawSubsystem; 
  private double Setpoint;
  public SetClawPID(ClawevatorSubsystem clawSubsystem, double Setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.clawSubsystem = clawSubsystem;
    this.Setpoint= Setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSubsystem.SetClawRotatePosition(Setpoint);

  }
}
