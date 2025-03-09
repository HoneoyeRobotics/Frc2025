// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreCoral2 extends SequentialCommandGroup {
  /** Creates a new AutoGooooooood. */
  public AutoScoreCoral2(DriveSubsystem driveSubsystem, ClawevatorSubsystem clawevatorSubsystem,
      VisionSubsystem visionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetClawevatorPosition(clawevatorSubsystem, ClawevatorPositions.Coral2),
        new SetClawPID(clawevatorSubsystem, RobotConstants.ClawRotateCoral2),
        new SetElevatorPID(clawevatorSubsystem, Constants.RobotConstants.ClawElevatorCoral2),
        new WaitUntilClawevatorAtPosition(clawevatorSubsystem).withTimeout(2),        
        new ForwardToLeftCoral(driveSubsystem, visionSubsystem, 0.25),
        new RunClaw(clawevatorSubsystem, () -> 0.1).withTimeout(2),
        new DriveRobot(driveSubsystem, -0.5, 0, 0).withTimeout(1)

    );

  }
}
