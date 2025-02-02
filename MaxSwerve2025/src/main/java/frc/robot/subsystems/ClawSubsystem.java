// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new Claw. */
  public ClawSubsystem() {
    lowerClawMotor.setNeutralMode(NeutralModeValue.Brake);
    lowerClawMotor.setInverted(true);

    upperClawMotor.setNeutralMode(NeutralModeValue.Brake);
    rotatePidController.setSetpoint(RotatePIDSetpoint);

  }

  private final SparkMax rotateMotor = new SparkMax(19, MotorType.kBrushless);
  private final TalonFX upperClawMotor = new TalonFX(7);
  private final TalonFX lowerClawMotor = new TalonFX(8);

  public void run(double speed) {
    upperClawMotor.set(speed);
    lowerClawMotor.set(speed);
  }

  public AnalogInput algaeSensor = new AnalogInput(1);

  public boolean algae = false;

  public boolean algaeCheck() {
    if (algaeSensor.getValue() < 500 && algaeSensor.getValue() > 230)
      algae = true;
    else
      algae = false;
    return algae;
  }

  private PIDController rotatePidController = new PIDController(2, 0, 0);
  private boolean EnableRotatePID = false;
  private double RotatePIDSetpoint = 0.50;

  public void ToggleRotatePID() {
    if (EnableRotatePID == true) {
      EnableRotatePID = false;
      rotateMotor.set(0);
    } else {
      EnableRotatePID = true;
    }
  }

  public void SetRotatePosition(double Setpoint) {

    // 0 to 0.3
    if (Setpoint < 0)
      Setpoint = 0;
    else if (Setpoint > 0.7)
      Setpoint = 0.7;
    RotatePIDSetpoint = Setpoint;
    rotatePidController.setSetpoint(RotatePIDSetpoint);

  }

  public void AdjustRotatePosition(double Setpoint) {

    RotatePIDSetpoint += Setpoint;
    // 0 to 0.3
    if (RotatePIDSetpoint < 0)
      RotatePIDSetpoint = 0;
    else if (RotatePIDSetpoint > 0.7)
      RotatePIDSetpoint = 0.7;

      
    rotatePidController.setSetpoint(RotatePIDSetpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double RotateSpeed = 0;
    double position = rotateMotor.getAbsoluteEncoder().getPosition();


    if (EnableRotatePID == true) {
      RotateSpeed = rotatePidController.calculate(position) * -1;
      rotateMotor.set(RotateSpeed);
      SmartDashboard.putNumber("Claw Speed", RotateSpeed);

    }
    SmartDashboard.putBoolean("Rotate PID", EnableRotatePID);
    SmartDashboard.putNumber("Claw Position",position );
    SmartDashboard.putNumber("Claw Setpoint", rotatePidController.getSetpoint());
    SmartDashboard.putBoolean("Algae", algaeCheck());
    SmartDashboard.putNumber("Algae Value", algaeSensor.getValue());
  }
}
