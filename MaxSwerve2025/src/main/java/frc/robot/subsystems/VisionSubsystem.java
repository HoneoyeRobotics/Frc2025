// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}
      
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
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

  }

  public double getX(){
    return tx.getDouble(0.0);
  }

  public boolean canSeeTag(){
    
    double area = ta.getDouble(0.0);
    return area > 0;
  }
  public boolean getOnTarget(){
    
    double x = tx.getDouble(0.0);
    double deadband = 0.264554 * x + 1.54865;
    return  x < deadband && x > -deadband; 
  }
}
