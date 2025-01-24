// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CalibrateCmd;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.TimedAutoDrive;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.subsystems.drivetrain.GyroADXRS450;
import frc.robot.subsystems.drivetrain.MAXSwerveModule;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

        public final Drivetrain driveSub;
        public final Gyro gyro;

        private final CommandXboxController driverController = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);

        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                gyro = new GyroADXRS450();
                gyro.calibrate();

                driveSub = new Drivetrain(
                                new MAXSwerveModule(DriveConstants.kFrontLeftDrivingCanId,
                                                DriveConstants.kFrontLeftTurningCanId,
                                                DriveConstants.kFrontLeftChassisAngularOffset),
                                new MAXSwerveModule(DriveConstants.kFrontRightDrivingCanId,
                                                DriveConstants.kFrontRightTurningCanId,
                                                DriveConstants.kFrontRightChassisAngularOffset),
                                new MAXSwerveModule(DriveConstants.kRearLeftDrivingCanId,
                                                DriveConstants.kRearLeftTurningCanId,
                                                DriveConstants.kBackLeftChassisAngularOffset),
                                new MAXSwerveModule(DriveConstants.kRearRightDrivingCanId,
                                                DriveConstants.kRearRightTurningCanId,
                                                DriveConstants.kBackRightChassisAngularOffset),
                                gyro);

                driveSub.setDefaultCommand(
                                new DriveCmd(
                                                driveSub,
                                                () -> MathUtil.applyDeadband(
                                                                -driverController.getRawAxis(
                                                                                OIConstants.kDriverControllerYAxis),
                                                                OIConstants.kDriveDeadband),
                                                () -> MathUtil.applyDeadband(
                                                                -driverController.getRawAxis(
                                                                                OIConstants.kDriverControllerXAxis),
                                                                OIConstants.kDriveDeadband),
                                                () -> MathUtil.applyDeadband(
                                                                -driverController.getRawAxis(
                                                                                OIConstants.kDriverControllerRotAxis),
                                                                OIConstants.kDriveDeadband)));
                configureAutos();
                configureBindings();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                driverController.b().onTrue(new CalibrateCmd(gyro));
        }

        private void configureAutos() {
                autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
                autoChooser.addOption("TimedAutoDrive", new TimedAutoDrive(driveSub));
                SmartDashboard.putData("Auto Routine", autoChooser.getSendableChooser());
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
