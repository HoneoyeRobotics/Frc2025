// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {


  }

  private DigitalInput leftDigitalInput = new DigitalInput(0);
  private DigitalInput rightDigitalInput = new DigitalInput(1);

  private SparkMax backClimber = new SparkMax(CanIDs.backClimber, MotorType.kBrushless);
  private SparkMax frontClimber = new SparkMax(CanIDs.frontClimber, MotorType.kBrushless);

  public void Move(double speed)
  {
    backClimber.set(speed);
    frontClimber.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LeftHook", leftDigitalInput.get());
    SmartDashboard.putBoolean("RightHook", rightDigitalInput.get());
  }
}
