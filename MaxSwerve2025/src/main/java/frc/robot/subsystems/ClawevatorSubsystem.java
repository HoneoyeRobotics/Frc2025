// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.RobotConstants;

public class ClawevatorSubsystem extends SubsystemBase {
  // properties

  private AnalogInput elevatorBottomSensor = new AnalogInput(0);
  private boolean EnablePID = false;
  private double ElevatorSetpoint = 0;
  private PIDController elevatorPIDContorller;
  private SparkMax leftElevator;
  private SparkMax rightElevator;
  private final SparkMax rotateMotor = new SparkMax(CanIDs.rotateClawMotor, MotorType.kBrushless);
  private final TalonFX upperClawMotor = new TalonFX(CanIDs.upperClawMotor);

  public AnalogInput algaeSensor = new AnalogInput(1);

  public boolean algae = false;

  private PIDController rotatePidController;;
  private boolean EnableRotatePID = false;
  private double RotatePIDSetpoint = RobotConstants.ClawRotateHome;

  private ClawevatorPositions SetPosition = ClawevatorPositions.Home;

  /** Creates a new ElevatorSubsystem. */
  public ClawevatorSubsystem() {

    upperClawMotor.setNeutralMode(NeutralModeValue.Brake);
    upperClawMotor.setInverted(true);
    rotatePidController = new PIDController(2, 0.6, 0.01);
    rotatePidController.setSetpoint(RotatePIDSetpoint);
    // rotatePidController.setTolerance(0.01);

    leftElevator = new SparkMax(CanIDs.leftElevator, MotorType.kBrushless);
    rightElevator = new SparkMax(CanIDs.rightElevator, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftElevator.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightElevator.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftElevator.getEncoder().setPosition(0.0);
    elevatorPIDContorller = new PIDController(0.1, 0, 0);
    elevatorPIDContorller.setSetpoint(0.0);
    elevatorPIDContorller.setTolerance(0.5);

  }

  // claw methods

  public void SetSetposition(ClawevatorPositions setposition) {
    SetPosition = setposition;
    SmartDashboard.putString("Position", SetPosition.toString());
  }

  public void runClaw(double speed, boolean EnablePID) {

    upperClawMotor.set(speed);

  }

  public boolean algaeCheck() {
    if (algaeSensor.getValue() < 500 && algaeSensor.getValue() > 230)
      algae = true;
    else
      algae = false;
    return algae;
  }

  public void ToggleClawRotatePID() {
    if (EnableRotatePID == true) {
      EnableRotatePID = false;
      rotateMotor.set(0);
    } else {
      EnableRotatePID = true;
    }
  }

  public void SetClawRotatePosition(double Setpoint) {

    // 0 to 0.3
    if (Setpoint < 0)
      Setpoint = 0;
    else if (Setpoint > 0.7)
      Setpoint = 0.7;
    RotatePIDSetpoint = Setpoint;
    rotatePidController.setSetpoint(RotatePIDSetpoint);

  }

  public void AdjustClawRotatePosition(double Setpoint) {

    RotatePIDSetpoint += Setpoint;
    // 0 to 0.3
    if (RotatePIDSetpoint < 0)
      RotatePIDSetpoint = 0;
    else if (RotatePIDSetpoint > 0.95)
      RotatePIDSetpoint = 0.95;

    rotatePidController.setSetpoint(RotatePIDSetpoint);
  }

  // elevator methods
  public void UpdateElevatorSetpoint(double setpoint) {
    ElevatorSetpoint += setpoint;
    elevatorPIDContorller.setSetpoint(ElevatorSetpoint);
  }

  public void SetElevatorSetpoint(double setpoint) {
    ElevatorSetpoint = setpoint;
    elevatorPIDContorller.setSetpoint(ElevatorSetpoint);
  }

  public void ToggleElevatorPID() {
    if (EnablePID == false) {
      EnablePID = true;
      elevatorPIDContorller.setSetpoint(leftElevator.getEncoder().getPosition());
      // elevatorPIDContorller.setP(Preferences.getDouble("ElevatorP", 0.2));
      // elevatorPIDContorller.setI(Preferences.getDouble("ElevatorI", 0));
      // elevatorPIDContorller.setD(Preferences.getDouble("ElevatorD", 0));

    } else {
      EnablePID = false;
      runElevator(0);
    }
  }

  public boolean elevatorAtBottom() {
    return elevatorBottomSensor.getVoltage() > 2;
  }

  public void runElevator(double speed) {
    leftElevator.set(speed);
    rightElevator.set(-speed);
    SmartDashboard.putNumber("ElevatorSpeed", speed);

  }

  public boolean SystemAtSetpoint() {
    return elevatorPIDContorller.atSetpoint() && rotatePidController.atSetpoint();
  }

  // overrides
  @Override
  public void periodic() {

    // claw functions
    double RotateSpeed = 0;
    double clawPosition = rotateMotor.getAbsoluteEncoder().getPosition();

    rotatePidController.setP(Preferences.getDouble("ClawP", 2));
    rotatePidController.setI(Preferences.getDouble("ClawI", 0.6));
    rotatePidController.setD(Preferences.getDouble("ClawD", 0.001));

    RotateSpeed = rotatePidController.calculate(clawPosition) * 1;
    rotateMotor.set(RotateSpeed);
    SmartDashboard.putNumber("Claw Speed", RotateSpeed);

    // elevator periodic functions

    if (elevatorAtBottom() && ElevatorSetpoint == 0 && leftElevator.getEncoder().getPosition() != 0) {
      leftElevator.getEncoder().setPosition(0);
      runElevator(0);
    }

    double elevatorPosition = leftElevator.getEncoder().getPosition();
    double speed = elevatorPIDContorller.calculate(elevatorPosition, ElevatorSetpoint);

    // //if close to home, cut power. Not needed with slower speed
    // if(ElevatorSetpoint == 0 && position > -2)
    // speed = 0;
    double maxSpeed = 0.7;
    // if moving down
    if (ElevatorSetpoint > elevatorPosition && speed > maxSpeed)
      speed = maxSpeed;

    // if moving up
    if (ElevatorSetpoint < elevatorPosition && speed < -maxSpeed)
      speed = -maxSpeed;
    // claw must be in the outer position for the elevator to move up or down aside
    // from being in the "final" position

    runElevator(speed);

    // dashboard reporting

    SmartDashboard.putNumber("Claw Position", clawPosition);
    SmartDashboard.putNumber("Claw Setpoint", rotatePidController.getSetpoint());
    SmartDashboard.putNumber("ElevatorSetpoint", elevatorPIDContorller.getSetpoint());
    SmartDashboard.putNumber("ElevatorPosition", leftElevator.getEncoder().getPosition());

    SmartDashboard.putBoolean("ElevatorAtSetpoint", elevatorPIDContorller.atSetpoint());
    SmartDashboard.putBoolean("RotateAtSetpoint", rotatePidController.atSetpoint());
  }

}
