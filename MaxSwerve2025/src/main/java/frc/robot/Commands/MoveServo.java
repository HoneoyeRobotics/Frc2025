// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveServo extends InstantCommand {
  private VisionSubsystem m_vision;
  public MoveServo(VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    SmartDashboard.putNumber("Servo", 0);
    
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_vision.moveServo(SmartDashboard.getNumber("Servo", 0));


  }
}
