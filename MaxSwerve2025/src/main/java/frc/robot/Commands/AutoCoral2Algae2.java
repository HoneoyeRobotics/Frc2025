// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCoral2Algae2 extends SequentialCommandGroup {
  /** Creates a new AutoGooooooood. */
  public AutoCoral2Algae2(DriveSubsystem driveSubsystem, ClawevatorSubsystem clawevatorSubsystem,
      VisionSubsystem visionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        // rotate to coral 2 scoring
        new SetClawPID(clawevatorSubsystem, RobotConstants.ClawRotateAlgae2),
        new SetElevatorPID(clawevatorSubsystem, Constants.RobotConstants.ClawElevatorAlgae2),

        // drive forward to X, Y coordinate for getting ready to score coral
        //new ForwardToAprilTag(driveSubsystem, visionSubsystem, true, false, 2, 2.8),        +
        new DriveRobot(driveSubsystem, 0.25,0.25,0).withTimeout(2),
        // run the spit out at X pct for 1 second
        new RunClawSet(clawevatorSubsystem, 0.10).withTimeout(1),
        // go to x,y coordinate to get ready to get algae while moving to algae 2
        //new ForwardToAprilTag(driveSubsystem, visionSubsystem, true, false, 0, 5),
        new DriveRobot(driveSubsystem, -0.25,-0.25,0).withTimeout(1),
        // run the pickup wheels while moving forward for x seconds picking up the ball

        new ParallelCommandGroup(
            new DriveRobot(driveSubsystem, 0.2,0,0).withTimeout(2),
            new RunClawSet(clawevatorSubsystem, 0.10).withTimeout(2)),
        // back up for 1 seconds (so)
        new DriveRobot(driveSubsystem, -0.5,0,0).withTimeout(1)

    );

  }
}
