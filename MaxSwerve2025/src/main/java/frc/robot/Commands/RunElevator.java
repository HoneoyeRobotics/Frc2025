// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {
  /** Creates a new RunElevator. */
  private final ClawevatorSubsystem ElevatorSubsystem;
  private final DoubleSupplier axis;
  public RunElevator(ClawevatorSubsystem ElevatorSubsystem, DoubleSupplier axis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ElevatorSubsystem = ElevatorSubsystem;
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

    double max = Preferences.getDouble("maxElevator", 0.1);

    if(speed > 0 && speed > max)
      speed = max;
    else if (speed < 0 && speed < -max)
      speed = -max;
    ElevatorSubsystem.runElevator(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    ElevatorSubsystem.runElevator(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
