// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftElevator = new SparkMax(34, MotorType.kBrushless);
    rightElevator = new SparkMax(33, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftElevator.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightElevator.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftElevator.getEncoder().setPosition(0.0);
    elevatorPIDContorller = new PIDController(0.4, 0, 0);
    elevatorPIDContorller.setSetpoint(0.0);
    elevatorPIDContorller.setTolerance(0.5);

  }
  private AnalogInput elevatorBottomSensor = new AnalogInput(0);
  private boolean EnablePID = false;
  private double ElevatorSetpoint = 0;
  private PIDController elevatorPIDContorller;
  private SparkMax leftElevator;
  private SparkMax rightElevator;

  public void UpdateSetpoint(double setpoint) {
    ElevatorSetpoint += setpoint;
    elevatorPIDContorller.setSetpoint(ElevatorSetpoint);
  }
  public void SetSetpoint(double setpoint) {
    ElevatorSetpoint = setpoint;
    elevatorPIDContorller.setSetpoint(ElevatorSetpoint);
  }
  public void TogglePID() {
    if (EnablePID == false) {
      EnablePID = true;
      elevatorPIDContorller.setSetpoint(leftElevator.getEncoder().getPosition());
      elevatorPIDContorller.setP(Preferences.getDouble("ElevatorP", 0.2));
      elevatorPIDContorller.setI(Preferences.getDouble("ElevatorI", 0));
      elevatorPIDContorller.setD(Preferences.getDouble("ElevatorD", 0));

    } else {
      EnablePID = false;
      run(0);
    }
  }

  public boolean elevatorAtBottom(){
    return elevatorBottomSensor.getVoltage() > 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LeftEleAmp", leftElevator.getOutputCurrent());
    SmartDashboard.putNumber("RightEleAmp", rightElevator.getOutputCurrent());
    SmartDashboard.putNumber("ElevatorSetpoint", elevatorPIDContorller.getSetpoint());

    SmartDashboard.putBoolean("Elevator at Bottom", elevatorAtBottom());

    SmartDashboard.putNumber("ElevatorPosition", leftElevator.getEncoder().getPosition());

    elevatorPIDContorller.setP(Preferences.getDouble("P", 0.2));

    elevatorPIDContorller.setI(Preferences.getDouble("I", 0));
    elevatorPIDContorller.setD(Preferences.getDouble("D", 0));

    if(elevatorAtBottom() && ElevatorSetpoint == 0 && leftElevator.getEncoder().getPosition() != 0) {
      leftElevator.getEncoder().setPosition(0);
      run(0);
    }

    if (EnablePID) {

      double position = leftElevator.getEncoder().getPosition();
      double speed = elevatorPIDContorller.calculate(position, ElevatorSetpoint);

      // //if close to home, cut power. Not needed with slower speed
      // if(ElevatorSetpoint == 0 && position > -2)
      //     speed = 0;

          //if moving down
      if(ElevatorSetpoint > position && speed > 0.33)
          speed = 0.33;

      //if moving up 
      if(ElevatorSetpoint < position && speed < -0.33)
          speed = -0.33;

      // claw must be in the outer position for the elevator to move up or down aside from being in the "final" position

      

      run(speed);
    }
  }

  public void run(double speed) {
    leftElevator.set(speed);
    rightElevator.set(-speed);
    SmartDashboard.putNumber("ElevatorSpeed", speed);

  }
}
