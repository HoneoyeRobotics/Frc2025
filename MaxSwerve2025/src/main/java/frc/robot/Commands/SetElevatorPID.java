// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClawevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorPID extends InstantCommand {
  private final ClawevatorSubsystem elevatorSubsystem;
  private final double position;
  public SetElevatorPID(ClawevatorSubsystem elevatorSubsystem, double Position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = Position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    double targetPosition = SmartDashboard.getNumber("Set Elevator  Setpoint", 0);
    if(position != 9999){
      targetPosition = position;
      SmartDashboard.putNumber("Set Elevator  Setpoint", targetPosition);
    }

    elevatorSubsystem.SetElevatorSetpoint(targetPosition);
  }
}
