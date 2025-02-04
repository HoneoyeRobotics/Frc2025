// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem robotDrive = new DriveSubsystem();
        private final VisionSubsystem vision = new VisionSubsystem();
        private final ClawSubsystem ClawSubsystem = new ClawSubsystem();
        private final ElevatorSubsystem ElevatorSubsystem = new ElevatorSubsystem();

        // The driver's controller
        CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                SmartDashboard.putData("Toggle Elevator PID", new TogglePID(ElevatorSubsystem));
                SmartDashboard.putData("Move Elevator Up", new UpdateElevatorPID(ElevatorSubsystem, -1));
                SmartDashboard.putData("Move Elevator Down", new UpdateElevatorPID(ElevatorSubsystem, 1));
                SmartDashboard.putData("Set Elevator PID", new SetElevatorPID(ElevatorSubsystem, 9999));

                SmartDashboard.putData("Move Elevator Home", new SetElevatorPID(ElevatorSubsystem, 0));
                
                SmartDashboard.putData("Move Elevator Top", new SetElevatorPID(ElevatorSubsystem, -90));
                SmartDashboard.putNumber("Set Elevator  Setpoint", 0);

                // Configure default commands
                robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> robotDrive.drive(
                                                                -MathUtil.applyDeadband(driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                // -MathUtil.applyDeadband(driverController.getRightX(),
                                                                // OIConstants.kDriveDeadband)
                                                                -MathUtil.applyDeadband(driverController
                                                                                .getRightTriggerAxis()
                                                                                - driverController
                                                                                                .getLeftTriggerAxis(),
                                                                                OIConstants.kDriveDeadband),
                                                                false,
                                                                driverController.leftBumper().getAsBoolean()),
                                                robotDrive));

                SmartDashboard.putData("Toggle Claw PID", new ToggleClawRotatePID(ClawSubsystem));
                SmartDashboard.putData("Rotate Claw Up", new RotateClaw(ClawSubsystem, 1));
                SmartDashboard.putData("Rotate Claw Down", new RotateClaw(ClawSubsystem, -1));

                SmartDashboard.putData("Claw Pickup", new SetClawPID(ClawSubsystem, 0.3));
                SmartDashboard.putData("Claw Drive w Ball", new SetClawPID(ClawSubsystem, 0.42));
                SmartDashboard.putData("Claw Drive", new SetClawPID(ClawSubsystem, 0.5));
                SmartDashboard.putData("Claw Shoot", new SetClawPID(ClawSubsystem, 0.5));


                ClawSubsystem.ToggleRotatePID();
                ElevatorSubsystem.TogglePID();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                driverController.rightBumper()
                                .whileTrue(new RunCommand(
                                                () -> robotDrive.setX(),
                                                robotDrive));
                driverController.back().onTrue(new ResetGyro(robotDrive));
                driverController.povLeft().whileTrue(new CenterOnAprilTag(robotDrive, vision, true)
                                .andThen(new RunCommand(() -> robotDrive.setX(), robotDrive).withTimeout(1)));
                driverController.povRight().whileTrue(new CenterOnAprilTag(robotDrive, vision, false)
                                .andThen(new RunCommand(() -> robotDrive.setX(), robotDrive).withTimeout(1)));

                driverController.axisGreaterThan(5, 0)
                                .whileTrue(new RunElevator(ElevatorSubsystem, () -> driverController.getRightY()));
                driverController.axisLessThan(5, 0)
                                .whileTrue(new RunElevator(ElevatorSubsystem, () -> driverController.getRightY()));

                driverController.x().whileTrue(new ShootClaw(ClawSubsystem));
                driverController.y().whileTrue(new RunClaw(ClawSubsystem, () -> -0.25));
                driverController.a().whileTrue(new RunClaw(ClawSubsystem, () -> 0.25));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                return new RunCommand(() -> robotDrive.drive(0, 0, 0, false, false), robotDrive);
                // // Create config for trajectory
                // TrajectoryConfig config = new TrajectoryConfig(
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(DriveConstants.kDriveKinematics);

                // // An example trajectory to follow. All units in meters.
                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(3, 0, new Rotation2d(0)),
                // config);

                // var thetaController = new ProfiledPIDController(
                // AutoConstants.kPThetaController, 0, 0,
                // AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // robotDrive::getPose, // Functional interface to feed supplier
                // DriveConstants.kDriveKinematics,

                // // Position controllers
                // new PIDController(AutoConstants.kPXController, 0, 0),
                // new PIDController(AutoConstants.kPYController, 0, 0),
                // thetaController,
                // robotDrive::setModuleStates,
                // robotDrive);

                // // Reset odometry to the starting pose of the trajectory.
                // robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // // Run path following command, then stop at the end.
                // return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0,
                // false, false));
        }
}
