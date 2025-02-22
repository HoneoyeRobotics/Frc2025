// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class VisionSubsystem extends SubsystemBase {

  private Servo servo = new Servo(9);
  private Servo DriveServo = new Servo(0);

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

    DriveServo.set(RobotConstants.DriveServoStraight);
    servoTimer = new Timer();
    servoTimer.start();
  }

  private double servoSetpoint = 0.3;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-drive");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  public void moveServo(double Degree) {
    servo.setPosition(Degree);
  }

  public void moveDriveServo(double Setpoint) {
    servoSetpoint += Setpoint;
    DriveServo.setPosition(servoSetpoint);
    SmartDashboard.putNumber("Drive Servo", servoSetpoint);
  }

  public void setDriveServo(double Setpoint) {
    DriveServo.setPosition(Setpoint);
    servoSetpoint = Setpoint;
    SmartDashboard.putNumber("Drive Servo", Setpoint);
  }

  Timer servoTimer;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putBoolean("canSeeTag", canSeeTag());
    SmartDashboard.putBoolean("onTarget", getOnTarget());

    SmartDashboard.putNumber("Current Angle", servo.getAngle());

    // can you find a tag?

    // if (canSeeTag()) {
    //   // y going negative is above, positive is below.
    //   // servo zero is up, 1 is down.

    //   if (servoTimer.hasElapsed(1)) {
    //     servoTimer.reset();
    //     // ok we are below it, move the angle up.
    //     if (y > 0.02) {

    //       servoSetpoint = servoSetpoint - 0.001;
    //       if (servoSetpoint < 0)
    //         servoSetpoint = 0;
    //       DriveServo.setPosition(servoSetpoint);
    //       System.out.println("Found, Moving up to " + servoSetpoint);
    //     } else if (y < -0.02) {
    //       // move it down..
    //       servoSetpoint = servoSetpoint + 0.001;
    //       if (servoSetpoint > 1)
    //         servoSetpoint = 1;
    //       // ok we are below it, move the angle up.
    //       DriveServo.setPosition(servoSetpoint);
    //       System.out.println("Found, Moving down to " + servoSetpoint);
    //     } else if (servoSetpoint != 0.6) {
    //       // move servo to middle
    //       servoSetpoint = 0.6;
    //       DriveServo.setPosition(0.6);
    //       System.out.println("Not Found, Changed to 0.6");

    //     }
    //   }
    // }
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public boolean canSeeTag() {

    double area = ta.getDouble(0.0);
    return area > 0;
  }

  public boolean getOnTarget() {

    if(canSeeTag() == false)
      return false;

    double x = tx.getDouble(-999);
    double deadband = 0.25;
    double target = -14.8;

    return x > target - deadband && 
          x < target + deadband;
  }
}
