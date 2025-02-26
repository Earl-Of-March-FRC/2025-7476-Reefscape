// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CalibrateCmd;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.TimedAutoDrive;
import frc.robot.commands.arm.ArmResetEncoderCmd;
import frc.robot.commands.arm.ArmSetPositionPIDCmd;
import frc.robot.commands.arm.ArmSetVelocityManualCmd;
import frc.robot.commands.indexer.IndexToSubsystemCmd;
import frc.robot.commands.indexer.IndexerSetVelocityManualCmd;
import frc.robot.commands.intake.IntakeSetVelocityManualCmd;
import frc.robot.commands.intake.IntakeStopCmd;
import frc.robot.commands.launcher.LauncherSetVelocityPIDCmd;
import frc.robot.commands.launcher.LauncherStopCmd;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.subsystems.drivetrain.GyroNavX;
import frc.robot.subsystems.drivetrain.MAXSwerveModule;
import frc.robot.subsystems.indexer.BeamBreakSensor;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.Launcher;

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

  private final ArmSubsystem armSub;
  private final IntakeSubsystem intakeSub;
  private final Indexer indexerSub;
  private final Launcher launcherSub;

  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    gyro = new GyroNavX();
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

    armSub = new ArmSubsystem(new SparkMax(ArmConstants.kMotorCanId, ArmConstants.kMotorType));

    intakeSub = new IntakeSubsystem(new SparkMax(IntakeConstants.kMotorCanId, IntakeConstants.kMotorType));

    indexerSub = new Indexer(
        new SparkMax(IndexerConstants.kMotorCanId, IndexerConstants.kMotorType),
        new BeamBreakSensor(IndexerConstants.kIntakeSensorChannel),
        new BeamBreakSensor(IndexerConstants.kLauncherSensorChannel));

    launcherSub = new Launcher(
        new SparkMax(LauncherConstants.kFrontCanId, LauncherConstants.kMotorType),
        new SparkMax(LauncherConstants.kBackCanId, LauncherConstants.kMotorType));

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

    // indexerSub.setDefaultCommand(
    // new IndexerSetVelocityManualCmd(indexerSub, () -> 0));

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
    // Manual arm control with
    armSub.setDefaultCommand(
        new ArmSetVelocityManualCmd(armSub, () -> MathUtil.applyDeadband(
            operatorController.getRawAxis(
                OIConstants.kOperatorArmManualAxis) * 0.5,
            OIConstants.kArmDeadband)));

    // Manual intake (arm roller) control with
    intakeSub.setDefaultCommand(
        new IntakeSetVelocityManualCmd(intakeSub,
            () -> MathUtil.applyDeadband(
                operatorController.getRawAxis(
                    OIConstants.kOperatorIntakeManualAxis),
                OIConstants.kIntakeDeadband)));

    driverController.b().onTrue(new CalibrateCmd(driveSub));

    // UNCOMMENT AFTER THE ARM IS TESTED
    // operatorController.button(7).onTrue(new ArmSetPositionPIDCmd(armSub,
    // ArmConstants.kAngleStowed));
    // operatorController.povDown().onTrue(new ArmSetPositionPIDCmd(armSub,
    // ArmConstants.kAngleGroundIntake));
    // operatorController.povRight().onTrue(new ArmSetPositionPIDCmd(armSub,
    // ArmConstants.kAngleL2));
    // operatorController.povLeft().onTrue(new ArmSetPositionPIDCmd(armSub,
    // ArmConstants.kAngleL3));
    // operatorController.povUp().onTrue(new ArmSetPositionPIDCmd(armSub,
    // ArmConstants.kAngleProcessor));

    operatorController.a()
        .whileTrue(new IntakeSetVelocityManualCmd(intakeSub, () -> IntakeConstants.kDefaultPercent));

    // operatorController.b().onTrue(new IntakeStopCmd(intakeSub));
    // operatorController.y().onTrue(new ArmResetEncoderCmd(armSub));
    driverController.x().onTrue(new IndexToSubsystemCmd(indexerSub, () -> -1));
    driverController.y().onTrue(new IndexToSubsystemCmd(indexerSub, () -> 1));
    driverController.rightTrigger().whileTrue(
        new LauncherSetVelocityPIDCmd(launcherSub, LauncherConstants.kVelocityFront, LauncherConstants.kVelocityBack));
    driverController.rightBumper().whileTrue(
        new IndexerSetVelocityManualCmd(indexerSub, () -> 1));
    driverController.leftBumper().whileTrue(
        new IndexerSetVelocityManualCmd(indexerSub, () -> -1));
    driverController.leftStick().onTrue(
        Commands.run(() -> {
          driveSub.isFieldRelative = !driveSub.isFieldRelative;
        }, driveSub));
  }

  /**
   * Use this method to define the autonomous command.
   */
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

  public CommandXboxController getOperatorController() {
    return operatorController;
  }
}
