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
public class AutoCoral3Algae1 extends SequentialCommandGroup {
  /** Creates a new AutoGooooooood. */
  public AutoCoral3Algae1(DriveSubsystem driveSubsystem, ClawevatorSubsystem clawevatorSubsystem,
      VisionSubsystem visionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoScoreCoral3(driveSubsystem, clawevatorSubsystem, visionSubsystem)

        ,

        new AutoGetAlgae1(driveSubsystem, clawevatorSubsystem, visionSubsystem));
  }
}
