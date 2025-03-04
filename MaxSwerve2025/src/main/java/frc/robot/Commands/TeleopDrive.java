// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  private final DriveSubsystem driveSubsystem;

  private final CommandXboxController driverController;

  /** Creates a new CenterOnAprilTag. */
  public TeleopDrive(DriveSubsystem driveSubsystem, CommandXboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.driverController = driverController;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

    double xSpeed = -MathUtil.applyDeadband(driverController.getLeftY(),OIConstants.kDriveDeadband);    
    double ySpeed = -MathUtil.applyDeadband(driverController.getLeftX(),OIConstants.kDriveDeadband);
    double zRot = -MathUtil.applyDeadband(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband);

    double RightxSpeed = -MathUtil.applyDeadband(driverController.getRightY(),OIConstants.kDriveDeadband);    
    double  RightySpeed = -MathUtil.applyDeadband(driverController.getRightX(),OIConstants.kDriveDeadband);


    boolean fieldRelative = true;

    //if the left stick is all zero, but the right stick is not, then use the robot relative drive.
    if(xSpeed == 0 && ySpeed == 0 && (RightxSpeed != 0 || RightySpeed != 0)){
      fieldRelative = false;
      xSpeed = RightxSpeed;
      ySpeed = RightySpeed;
    }


    //if you are pushing on the left stick, modify based ont he limelight.
    if(driverController.start().getAsBoolean() == true) {

      //strafe
      double strafekP = .035;
      double strafeVelocity = LimelightHelpers.getTX("limelight-drive") * strafekP;
      double speedModifier =  Math.abs(xSpeed);
      if(speedModifier < 0.10 && speedModifier > -0.10)
        speedModifier = 0.10 * (speedModifier < 0 ? -1 : 0);
      strafeVelocity *= speedModifier;

    //invert since tx is positive when the target is to the right of the crosshair
    
      ySpeed = strafeVelocity;
      //zRot = rot_limelight;

      // double forward_limelight = limelight_range_proportional();
      // xSpeed = forward_limelight;
      double tag = LimelightHelpers.getFiducialID("limelight-drive");
      double targetAngle = 0;
      if(tag == 6)
        targetAngle = 180;
      else if (tag == 2)
        targetAngle = -90;
      int heading = (int)driveSubsystem.getHeading();
      int remainder = (int)(Math.abs(heading) % targetAngle );
      int closest = (int)(heading - (heading % targetAngle));
      // if(heading < 0){
      //   closest *= -1;
      // }
      System.out.println("heading: " + heading + "; remainder: " + remainder 
      + "closest:" + closest);
      
      //we want to be 180, find where we are



      //if the remainder is more than half of the current setpoint, then we want to go back rather than forward.

      double rotatekP = .035;
      double rotateVelocity = rotateDifference * strafekP; 
      double rotatespeedModifier =  Math.abs(xSpeed);
      if(rotatespeedModifier < 0.10 && rotatespeedModifier > -0.10)
        rotatespeedModifier = 0.10 * (rotatespeedModifier < 0 ? -1 : 0);
        
      rotateVelocity *= rotatespeedModifier;
    }

    //normal driving;
    driveSubsystem.drive(
        xSpeed,
        ySpeed,        
        zRot,
        fieldRelative,
        driverController.leftBumper().getAsBoolean()

    );
  }



  public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight-drive") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= 1;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-drive") * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
