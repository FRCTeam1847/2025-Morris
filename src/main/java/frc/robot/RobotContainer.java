// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Levels;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),
        // "swerve");
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve/kraken"));
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem(elevatorSubsystem,
                        intakeSubsystem);
        private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
        private final Light lights = new Light();

        ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

        private final SendableChooser<Command> autoChooser;
        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandPS5Controller controller = new CommandPS5Controller(0);

        private Command activeScoreCommand = null;

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> controller.getLeftY() * -1,
                        () -> controller.getLeftX() * -1)
                        .withControllerRotationAxis(controller::getRightX)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.5)
                        .allianceRelativeControl(true);

        SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX())
                        .withControllerRotationAxis(() -> controller.getRightX())
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.5)
                        .allianceRelativeControl(true);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                configureDefaultCommand();
                registerNamedCommands();

                drivebase.setupPathPlanner();
                autoChooser = AutoBuilder.buildAutoChooser();
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                driverTab.add("Auto Chooser", autoChooser)
                                .withSize(2, 1)
                                .withPosition(3, 0)
                                .withWidget(BuiltInWidgets.kComboBoxChooser);
                // SmartDashboard.putData("Auto Chooser", autoChooser);

                driverTab.add("Main", "http://limelight.local:5800/stream.mjpg")
                                .withSize(3, 4)
                                .withPosition(0, 0).withWidget(BuiltInWidgets.kCameraStream);

                driverTab.add("Back", "http://limelight-main.local:5800/stream.mjpg")
                                .withSize(3, 4)
                                .withPosition(5, 0).withWidget(BuiltInWidgets.kCameraStream);
                // driverTab.add("Field", SmartDashboard.getData("Field"))
                //                 .withSize(8, 5)
                //                 .withPosition(0, 4).withWidget(BuiltInWidgets.kField);
        }

        private void configureDefaultCommand() {
        }

        private void registerNamedCommands() {
                NamedCommands.registerCommand(
                                "Home",
                                new InstantCommand(
                                                () -> manipulatorSubsystem.setLevel(Levels.Home),
                                                manipulatorSubsystem));
                NamedCommands.registerCommand("L1", scoreCommandWithTracking(Levels.L1));
                NamedCommands.registerCommand("L2", scoreCommandWithTracking(Levels.L2));
                NamedCommands.registerCommand("L3", scoreCommandWithTracking(Levels.L3));
                NamedCommands.registerCommand("L4", scoreCommandWithTracking(Levels.L4));
                // NamedCommands.registerCommand("resetElevator", new InstantCommand(() ->
                // manipulatorSubsystem.rest ));
                NamedCommands.registerCommand("L2Only",
                                new RunCommand(() -> manipulatorSubsystem.SetJustLevel(Levels.L2)));
                NamedCommands.registerCommand("L3Only",
                                manipulatorSubsystem.SetJustLevel(Levels.L3));
                NamedCommands.registerCommand("L4Only",
                                new InstantCommand(() -> manipulatorSubsystem.SetJustLevel(Levels.L4)));
                 NamedCommands.registerCommand("HasCoral",
                                new InstantCommand(() -> manipulatorSubsystem.hasInnerCoral()));

                NamedCommands.registerCommand(
                                "Intake", manipulatorSubsystem.intakeCommand());
                NamedCommands.registerCommand(
                                "Release", manipulatorSubsystem.releaseCommand());
                NamedCommands.registerCommand(
                                "IntakeStop", manipulatorSubsystem.intakeStopCommand());
                NamedCommands.registerCommand("ClimberUp", climberSubsystem.moveForwardCommand());
                NamedCommands.registerCommand("ClimberDown", climberSubsystem.moveBackwardCommand());
                NamedCommands.registerCommand("ClimberStop",
                                climberSubsystem.stopCommand());
                NamedCommands.registerCommand("CancelCommand", new InstantCommand(() -> cancelActiveScoreCommand()));
                NamedCommands.registerCommand("AlignRight",
                                new AlignToReefTagRelative(true, drivebase).withTimeout(Constants.ALIGN_TIMEOUT));
                NamedCommands.registerCommand("AlignLeft",
                                new AlignToReefTagRelative(false, drivebase).withTimeout(Constants.ALIGN_TIMEOUT));
                NamedCommands.registerCommand("ResetPose", new InstantCommand(
                                () -> manipulatorSubsystem.resetElevator()));
        }

        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocitySim);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }

                controller.R2().whileTrue(NamedCommands.getCommand("Intake"))
                                .onFalse(NamedCommands.getCommand("IntakeStop"));

                controller.L2().whileTrue(NamedCommands.getCommand("Release"))
                                .onFalse(NamedCommands.getCommand("IntakeStop"));
                controller.L1().onTrue(NamedCommands.getCommand("Home"));

                controller.cross().whileTrue(NamedCommands.getCommand("L1"))
                                .onFalse(NamedCommands.getCommand("CancelCommand"));

                controller.circle().whileTrue(NamedCommands.getCommand("L2"))
                                .onFalse(NamedCommands.getCommand("CancelCommand"));

                controller.triangle().whileTrue(NamedCommands.getCommand("L4"))
                                .onFalse(NamedCommands.getCommand("CancelCommand"));

                controller.square().whileTrue(NamedCommands.getCommand("L3"))
                                .onFalse(NamedCommands.getCommand("CancelCommand"));

                controller.povUp().whileTrue(NamedCommands.getCommand("ClimberUp"))
                                .onFalse(NamedCommands.getCommand("ClimberStop"));
                controller.povDown().whileTrue(NamedCommands.getCommand("ClimberDown"))
                                .onFalse(NamedCommands.getCommand("ClimberStop"));

                controller.povRight().whileTrue(NamedCommands.getCommand("AlignRight"));
                controller.povLeft().whileTrue(NamedCommands.getCommand("AlignLeft"));
                controller.R1().onTrue(NamedCommands.getCommand("ResetPose").ignoringDisable(true));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        private void cancelActiveScoreCommand() {
                if (activeScoreCommand != null && activeScoreCommand.isScheduled()) {
                        System.out.println("Cancelling ScoreAtLevelCommand...");
                        activeScoreCommand.cancel();
                        activeScoreCommand = null;
                }
        }

        private Command scoreCommandWithTracking(Levels level) {
                /** TEST THIS */
                // return new ProxyCommand(() -> {
                // cancelActiveScoreCommand(); // Cancel any running score command first
                // Command cmd = manipulatorSubsystem.ScoreAtLevelParallelCommand(level)
                // .finallyDo(() -> {
                // System.out.println("Finished ScoreAtLevel for " + level);
                // activeScoreCommand = null;
                // });
                // activeScoreCommand = cmd;
                // return cmd;
                // });

                return new DeferredCommand(() -> {
                        cancelActiveScoreCommand();
                        Command cmd = manipulatorSubsystem.ScoreAtLevelParallelCommand(level)
                                        .finallyDo(() -> {
                                                System.out.println("Finished ScoreAtLevel for " + level);
                                                activeScoreCommand = null;
                                        });
                        activeScoreCommand = cmd;
                        return cmd;
                }, Set.of(manipulatorSubsystem));
        }

        public void setElevatorIdleMode(boolean brakemode) {
                elevatorSubsystem.setBrakeMode(brakemode);
        }
}
