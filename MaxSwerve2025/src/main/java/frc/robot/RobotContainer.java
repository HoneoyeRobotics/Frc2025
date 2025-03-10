// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.*;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

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
        Thread m_visionThread;
        // The robot's subsystems
        private final DriveSubsystem robotDrive = new DriveSubsystem();
        private final VisionSubsystem vision = new VisionSubsystem();
        private final ClawevatorSubsystem ClawevatorSubsystem = new ClawevatorSubsystem();
        private final Climber climber = new Climber();
        private SendableChooser<Command> auto = new SendableChooser<>();

        // The driver's controller
        CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandJoystick buttonBoard = new CommandJoystick(1);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */

        public void teleopInit() {

                // vision.moveServo(133);
                // vision.moveServo(0);
        }

        public void DoTheCamera() {
                m_visionThread = new Thread(
                                () -> {
                                        // Get the UsbCamera from CameraServer
                                        UsbCamera camera = CameraServer.startAutomaticCapture();
                                        // Set the resolution
                                        camera.setResolution(640, 480);

                                        // Get a CvSink. This will capture Mats from the camera
                                        CvSink cvSink = CameraServer.getVideo();
                                        // Setup a CvSource. This will send images back to the Dashboard
                                        CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

                                        // Mats are very memory expensive. Lets reuse this Mat.
                                        Mat mat = new Mat();

                                        // This cannot be 'true'. The program will never exit if it is. This
                                        // lets the robot stop this thread when restarting robot code or
                                        // deploying.
                                        while (!Thread.interrupted()) {
                                                // Tell the CvSink to grab a frame from the camera and put it
                                                // in the source mat. If there is an error notify the output.
                                                if (cvSink.grabFrame(mat) == 0) {
                                                        // Send the output the error.
                                                        outputStream.notifyError(cvSink.getError());
                                                        // skip the rest of the current iteration
                                                        continue;
                                                }
                                                // Put a rectangle on the image
                                                // Imgproc.rectangle(
                                                //                 mat, new Point(100, 100), new Point(400, 400),
                                                //                 new Scalar(255, 255, 255), 5);

                                                Imgproc.line(
                                                                mat, new Point(250, 50), new Point(210, 140),
                                                                new Scalar(0, 0, 200), 4);
                                                // Give the output stream a new image to display
                                                outputStream.putFrame(mat);
                                        }
                                });
                m_visionThread.setDaemon(true);
                m_visionThread.start();
        }

        public RobotContainer() {
                DoTheCamera();
                // UsbCamera camera = new UsbCamera("Climb", 1);
                // camera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 15);
                // CameraServer.startAutomaticCapture(camera);
                // camera.setExposureManual(10);
                // camera.setWhiteBalanceManual(50);
                // Configure the button bindings`

                // camera2.setResolution(640, 480 );
                // camera2.setVideoMode(PixelFormat.kMJPEG, 640, 480, 15);
                configureButtonBindings();

                // Configure default commands
                robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new TeleopDrive(robotDrive, driverController, vision, ClawevatorSubsystem));

                SmartDashboard.putData("Auto Test", new AutoCoral2Algae2(robotDrive, ClawevatorSubsystem, vision));

                // SmartDashboard.putData("Toggle Claw PID", new
                // ToggleClawRotatePID(ClawevatorSubsystem));
                SmartDashboard.putData("Rotate Claw Up", new RotateClaw(ClawevatorSubsystem, 1));
                SmartDashboard.putData("Rotate Claw Down", new RotateClaw(ClawevatorSubsystem, -1));

                // SmartDashboard.putData("Toggle Elevator PID", new
                // TogglePID(ClawevatorSubsystem));
                SmartDashboard.putData("Move Elevator Up", new UpdateElevatorPID(ClawevatorSubsystem, 1));
                SmartDashboard.putData("Move Elevator Down", new UpdateElevatorPID(ClawevatorSubsystem, -1));
                // SmartDashboard.putData("Set Elevator PID", new
                // SetElevatorPID(ClawevatorSubsystem, 9999));
                // SmartDashboard.putNumber("Set Elevator Setpoint", 0);

                // SmartDashboard.putData("Claw Pickup",
                // new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotatePickup));
                // SmartDashboard.putData("Claw Drive ",
                // new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateDrive));
                // SmartDashboard.putData("Claw Shoot",
                // new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateShoot));
                // SmartDashboard.putData("Claw Home", new SetClawPID(ClawevatorSubsystem,
                // RobotConstants.ClawRotateHome));
                // SmartDashboard.putData("Claw Algae 1",
                // new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateAlgae1));
                // SmartDashboard.putData("Claw Algae 2",
                // new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateAlgae2));

                // SmartDashboard.putData("Move Elevator Home",
                // new SetElevatorPID(ClawevatorSubsystem, RobotConstants.ClawElevatorHome));
                // SmartDashboard.putData("Move Elevator Top",
                // new SetElevatorPID(ClawevatorSubsystem,
                // Constants.RobotConstants.ClawElevatorShoot));
                // SmartDashboard.putData("Move Elevator Algae 1",
                // new SetElevatorPID(ClawevatorSubsystem,
                // Constants.RobotConstants.ClawElevatorAlgae1));
                // SmartDashboard.putData("Move Elevator Algae 2",
                // new SetElevatorPID(ClawevatorSubsystem,
                // Constants.RobotConstants.ClawElevatorAlgae2));

                // SmartDashboard.putData("ServoUp", new MoveDriveServo(vision, -0.01));

                // SmartDashboard.putData("ServoStraight", new SetDriveServo(vision,
                // RobotConstants.DriveServoStraight));

                // SmartDashboard.putData("ServoBottom", new SetDriveServo(vision,
                // RobotConstants.DriveServoBottom));

                NamedCommands.registerCommand("Claw Pick Up", new WaitCommand(0.1));
                NamedCommands.registerCommand("Shoot", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm L-1", new WaitCommand(0.1));
                NamedCommands.registerCommand("Claw Drive", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm L-2", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm L-3", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm Floor", new WaitCommand(0.1));
                NamedCommands.registerCommand("Arm Shoot", new WaitCommand(0.1));
                NamedCommands.registerCommand("AutoGetAlgae2",
                                new AutoGetAlgae2(robotDrive, ClawevatorSubsystem, vision));
                NamedCommands.registerCommand("AutoGetAlgae1",
                                new AutoGetAlgae1(robotDrive, ClawevatorSubsystem, vision));

                SmartDashboard.putData("Auto Mode", auto);
                auto.addOption("Auto 1", new PathPlannerAuto("Auto 1"));
                auto.addOption("Auto 2", new PathPlannerAuto("Auto 2"));
                auto.addOption("Is gyro weird again", new PathPlannerAuto("Is gyro weird again"));
                auto.addOption("GrabAlgae1", new PathPlannerAuto("GrabAlgae"));
                auto.addOption("AutoCoral2Algae2", new AutoCoral2Algae2(robotDrive, ClawevatorSubsystem, vision));
                auto.addOption("AutoCoral3Algae1", new AutoCoral3Algae1(robotDrive, ClawevatorSubsystem, vision));

                ClawevatorSubsystem.ToggleClawRotatePID();
                ClawevatorSubsystem.ToggleElevatorPID();
                SmartDashboard.putData("ForwardToAlgae", new ForwardToAlgae(robotDrive, vision, 0.25));
                SmartDashboard.putData("ForwardToLeftCoral", new ForwardToLeftCoral(robotDrive, vision, 0.25));
                SmartDashboard.putData("AutoGetAlgae2", new AutoGetAlgae2(robotDrive, ClawevatorSubsystem, vision));
                SmartDashboard.putData("AutoGetAlgae1", new AutoGetAlgae1(robotDrive, ClawevatorSubsystem, vision));
                SmartDashboard.putData("AutoScoreCoral2", new AutoScoreCoral2(robotDrive, ClawevatorSubsystem, vision));
                SmartDashboard.putData("AutoScoreCoral3", new AutoScoreCoral3(robotDrive, ClawevatorSubsystem, vision));
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
                driverController.start().whileTrue(new RunCommand(() -> robotDrive.setX(), robotDrive));
                driverController.back().onTrue(new ResetGyro(robotDrive));
                // driverController.povLeft().whileTrue(new CenterOnAprilTag(robotDrive, vision,
                // true)
                // .andThen(new RunCommand(() -> robotDrive.setX(),
                // robotDrive).withTimeout(1)));
                driverController.povRight().whileTrue(new CenterOnAprilTag(robotDrive, vision, false)
                                .andThen(new RunCommand(() -> robotDrive.setX(), robotDrive).withTimeout(1)));

                driverController.povUp().whileTrue(new ForwardToAprilTag(robotDrive, vision, false, true, 0, 2.8)
                                .andThen(new RunCommand(() -> robotDrive.setX(), robotDrive).withTimeout(1)));

                driverController.povLeft().whileTrue(new ForwardToAprilTag(robotDrive, vision, true, false, 4, -8.5)
                                .andThen(new RunCommand(() -> robotDrive.setX(), robotDrive).withTimeout(1)));
                driverController.povDown().whileTrue(new ForwardToAprilTag(robotDrive, vision, true, false, -8.5, -9.32)
                                .andThen(new RunCommand(() -> robotDrive.setX(), robotDrive).withTimeout(1)));

                // driverController.axisGreaterThan(5, 0)
                // .whileTrue(new RunElevator(ClawevatorSubsystem, () ->
                // driverController.getRightY()));
                // driverController.axisLessThan(5, 0)
                // .whileTrue(new RunElevator(ClawevatorSubsystem, () ->
                // driverController.getRightY()));

                driverController.x().whileTrue(new ShootClaw(ClawevatorSubsystem));
                driverController.y().whileTrue(new RunClaw(ClawevatorSubsystem, () -> -0.25));
                driverController.a().whileTrue(new RunClaw(ClawevatorSubsystem, () -> 0.5));
                driverController.b().whileTrue(new RunClaw(ClawevatorSubsystem, () -> 0.1));
                // driverController.start().whileTrue(new DoTheClimb(climber));

                // coral level 3
                buttonBoard.button(4).onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateCoral3)
                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                Constants.RobotConstants.ClawElevatorCoral3))
                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem, ClawevatorPositions.Coral3)));
                // coral level 2
                buttonBoard.button(5).onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateCoral2)
                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                Constants.RobotConstants.ClawElevatorCoral2))
                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem, ClawevatorPositions.Coral2)));
                // algae level 1
                buttonBoard.button(8).onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateAlgae1)
                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                Constants.RobotConstants.ClawElevatorAlgae1))
                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem, ClawevatorPositions.Algae1)));
                // algae level 2
                buttonBoard.button(7).onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateAlgae2)
                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                Constants.RobotConstants.ClawElevatorAlgae2))
                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem, ClawevatorPositions.Algae2)));
                // drive
                buttonBoard.button(10).onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateDrive)
                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                Constants.RobotConstants.ClawElevatorDrive))
                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem, ClawevatorPositions.Drive)));
                // home
                buttonBoard.button(2).onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateHome)
                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                Constants.RobotConstants.ClawElevatorHome))
                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem, ClawevatorPositions.Home)));
                // pickup
                buttonBoard.button(9).onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotatePickup)
                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                Constants.RobotConstants.ClawElevatorPickup))
                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem, ClawevatorPositions.Pickup)));
                // shoot
                buttonBoard.button(3).onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateShoot)
                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                Constants.RobotConstants.ClawElevatorShoot))
                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem, ClawevatorPositions.Shoot)));

                // stacked
                buttonBoard.button(6).onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateAlgaeStacked)
                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                Constants.RobotConstants.ClawElevatorAlgaeStacked))
                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem, ClawevatorPositions.Stack)));

                buttonBoard.button(1).whileTrue(new DoTheClimb(climber));

                // feed
                buttonBoard.axisLessThan(1, 0)
                                .onTrue(new SetClawPID(ClawevatorSubsystem, RobotConstants.ClawRotateCoralFeed)
                                                .andThen(new SetElevatorPID(ClawevatorSubsystem,
                                                                Constants.RobotConstants.ClawElevatorCoralFeed))
                                                .andThen(new SetClawevatorPosition(ClawevatorSubsystem,
                                                                ClawevatorPositions.Feeder)));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                return auto.getSelected();

        }
}
