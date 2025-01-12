// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new Claw. */
  public ClawSubsystem() {
    lowerClawMotor.setNeutralMode(NeutralModeValue.Brake);

    upperClawMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  private final TalonFX upperClawMotor = new TalonFX(7);
  private final TalonFX lowerClawMotor = new TalonFX(8);

  public void run(double speed){
    upperClawMotor.set(-speed);
    lowerClawMotor.set(speed);
  }
  

  public AnalogInput algaeSensor = new AnalogInput(0);

  public boolean algae = false;
  public boolean algaeCheck() {
    if (algaeSensor.getValue() < 500 && algaeSensor.getValue() > 230)
      algae = true;
    else
      algae = false;
    return algae;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Algae", algaeCheck());
    SmartDashboard.putNumber("Algae Value", algaeSensor.getValue() );
  }
}
