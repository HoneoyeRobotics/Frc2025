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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.auto.NamedCommands;

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
        private final Climber climber = new Climber();
        private SendableChooser<Command> auto = new SendableChooser<>();

        // The driver's controller
        CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */

         public void teleopInit() {
                vision.moveServo(133);
                vision.UpServo(0);
         }

        public RobotContainer() {
                // Configure the button bindings`
                configureButtonBindings();

                SmartDashboard.putData("Toggle Elevator PID", new TogglePID(ElevatorSubsystem));
                SmartDashboard.putData("Move Elevator Up", new UpdateElevatorPID(ElevatorSubsystem, 1));
                SmartDashboard.putData("Move Elevator Down", new UpdateElevatorPID(ElevatorSubsystem, -1));
                SmartDashboard.putData("Set Elevator PID", new SetElevatorPID(ElevatorSubsystem, 9999));

                SmartDashboard.putData("Move Elevator Home", new SetElevatorPID(ElevatorSubsystem, 0));
                
                SmartDashboard.putData("Move Elevator Top", new SetElevatorPID(ElevatorSubsystem, 94));
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
                                                              driverController.leftBumper().getAsBoolean()
                                                              
                                                               ),
                                                robotDrive)  );

                SmartDashboard.putData("Toggle Claw PID", new ToggleClawRotatePID(ClawSubsystem));
                SmartDashboard.putData("Rotate Claw Up", new RotateClaw(ClawSubsystem, 1));
                SmartDashboard.putData("Rotate Claw Down", new RotateClaw(ClawSubsystem, -1));

                SmartDashboard.putData("Claw Pickup", new SetClawPID(ClawSubsystem, RobotConstants.ClawRotatePickup));
                SmartDashboard.putData("Claw Drive w Ball", new SetClawPID(ClawSubsystem, RobotConstants.ClawWithBall));
                SmartDashboard.putData("Claw Shoot", new SetClawPID(ClawSubsystem, RobotConstants.ClawRotateUp));
                
                SmartDashboard.putData("moveServo", new MoveServo(vision));
                SmartDashboard.putData("UpServo", new UpServo(vision));

                SmartDashboard.putData("Auto Mode", auto);
               auto.addOption("Auto 1", new PathPlannerAuto("Auto 1"));
               auto.addOption("Auto 2", new PathPlannerAuto("Auto 2"));
               auto.addOption("Is gyro weird again", new PathPlannerAuto ("Is gyro weird again"));
               
                NamedCommands.registerCommand("Claw Pick Up", new WaitCommand(0.1));
                NamedCommands.registerCommand("Shoot", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm L-1", new WaitCommand(0.1));
                NamedCommands.registerCommand("Claw Drive", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm L-2", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm L-3", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm Floor", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm Shoot", new WaitCommand(0.1));

                ClawSubsystem.ToggleRotatePID();
                ElevatorSubsystem.TogglePID();

                SmartDashboard.putData(climber);
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
                driverController.back().onTrue(new ResetGyro(robotDrive)
                );
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
                driverController.b().whileTrue(new RunClaw(ClawSubsystem, () -> 0.05));
                driverController.start().whileTrue(new DoTheClimb(climber));
        }       

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                return auto.getSelected();
                //return new RunCommand(() -> robotDrive.drive(0, 0, 0, false, false), robotDrive);
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
