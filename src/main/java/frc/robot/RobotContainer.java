// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.Constants.DriveConstants.LaunchingDistances;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.StripAlgaeCmd;
import frc.robot.commands.arm.ArmSetPositionPIDCmd;
import frc.robot.commands.arm.ArmSetVelocityManualCmd;
import frc.robot.commands.indexer.IndexToBeamBreakCmd;
import frc.robot.commands.indexer.IndexerSetVelocityManualCmd;
import frc.robot.commands.intake.IntakeSetVelocityManualCmd;
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

  public final ArmSubsystem armSub;
  private final IntakeSubsystem intakeSub;
  private final Indexer indexerSub;
  private final Launcher launcherSub;

  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  private final SlewRateLimiter intakeLimiter = new SlewRateLimiter(5);

  // Register Named Commands

  private LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    gyro = new GyroNavX();

    armSub = new ArmSubsystem(new SparkMax(ArmConstants.kMotorCanId, ArmConstants.kMotorType),
        ArmConstants.kLimitSwitchChannel);
    armSub.calibrate();

    intakeSub = new IntakeSubsystem(new SparkMax(IntakeConstants.kMotorCanId, IntakeConstants.kMotorType));

    indexerSub = new Indexer(
        new SparkMax(IndexerConstants.kMotorCanId, IndexerConstants.kMotorType),
        new BeamBreakSensor(IndexerConstants.kIntakeSensorChannel),
        new BeamBreakSensor(IndexerConstants.kLauncherSensorChannel));

    launcherSub = new Launcher(
        new SparkMax(LauncherConstants.kFrontCanId, LauncherConstants.kMotorType),
        new SparkMax(LauncherConstants.kBackCanId, LauncherConstants.kMotorType));

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
        gyro, launcherSub::isUsingHighVelocities);

    // Register named Commands
    NamedCommands.registerCommand("Calibrate", new CalibrateGyroCmd(driveSub));
    NamedCommands.registerCommand("ArmL2", new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleL2));
    NamedCommands.registerCommand("ArmL3", new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleL3));
    NamedCommands.registerCommand("ArmStow", new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleStowed));
    NamedCommands.registerCommand("StripAlgae", new StripAlgaeCmd(intakeSub, armSub));
    NamedCommands.registerCommand("LauncherIntake",
        new LauncherSetVelocityPIDCmd(launcherSub, () -> -launcherSub.getPreferredFrontVelocity(),
            () -> -launcherSub.getPreferredBackVelocity()));
    NamedCommands.registerCommand("RevLauncher",
        new LauncherSetVelocityPIDCmd(launcherSub, () -> launcherSub.getPreferredFrontVelocity(),
            () -> launcherSub.getPreferredBackVelocity()));
    NamedCommands.registerCommand("StopLauncher", new LauncherStopCmd(launcherSub));
    NamedCommands.registerCommand("Launch", new IndexerSetVelocityManualCmd(indexerSub, () -> 1));
    NamedCommands.registerCommand("StopIndexer", new IndexerSetVelocityManualCmd(indexerSub, () -> 0));
    NamedCommands.registerCommand("RunIntake", new IntakeSetVelocityManualCmd(intakeSub,
        () -> -1));
    NamedCommands.registerCommand("StopIntake", new IntakeSetVelocityManualCmd(intakeSub,
        () -> 0));
    NamedCommands.registerCommand("IndexerBack",
        new IndexerSetVelocityManualCmd(indexerSub, () -> -1).until(() -> !indexerSub.getIntakeSensor()));
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

    // SET DEFAULT COMMANDS

    // Drive with
    driveSub.setDefaultCommand(
        new DriveSqrtCmd(
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

    // Manual arm control with
    armSub.setDefaultCommand(
        new ArmSetVelocityManualCmd(armSub, () -> MathUtil.applyDeadband(
            operatorController.getRawAxis(OIConstants.kOperatorArmManualAxis),
            armSub.getIsUsingPid() ? OIConstants.kArmManualDeadband : OIConstants.kArmDeadband)
            * ArmConstants.kMaxArmManualSpeedPercent));

    // Manual intake (arm roller) control with
    intakeSub.setDefaultCommand(
        new IntakeSetVelocityManualCmd(intakeSub,
            () -> intakeLimiter.calculate(MathUtil.applyDeadband(
                operatorController.getRawAxis(
                    OIConstants.kOperatorIntakeManualAxis),
                OIConstants.kIntakeDeadband))));

    // DRIVER CONTROLLER

    // Drive commands
    driverController.b().onTrue(new CalibrateGyroCmd(driveSub));
    driverController.x().whileTrue(new AlignReefWithDriveBangBangCmd(
        driveSub,
        () -> MathUtil.applyDeadband(
            driverController.getRawAxis(
                OIConstants.kDriverControllerYAxis),
            OIConstants.kDriveDeadband),
        () -> MathUtil.applyDeadband(
            driverController.getRawAxis(
                OIConstants.kDriverControllerXAxis),
            OIConstants.kDriveDeadband)));

    // Move to barge launching zone, facing in the specified direction
    driverController.povLeft().whileTrue(
        new MoveToPoseBangBangCmd(driveSub,
            () -> driveSub.getBargeTargetPose(LaunchingDistances.kTargetBargeAngleLeft),
            false));
    driverController.povUp().whileTrue(
        new MoveToPoseBangBangCmd(driveSub,
            () -> driveSub.getBargeTargetPose(LaunchingDistances.kTargetBargeAngleStraight),
            false));
    driverController.a().whileTrue(
        new MoveToPoseBangBangCmd(driveSub,
            () -> driveSub.getBargeTargetPose(LaunchingDistances.kTargetBargeAngleStraight),
            false));
    driverController.povRight().whileTrue(
        new MoveToPoseBangBangCmd(driveSub,
            () -> driveSub.getBargeTargetPose(LaunchingDistances.kTargetBargeAngleRight),
            false));

    // Toggle field or robot oriented drive
    driverController.leftStick().onTrue(
        Commands.runOnce(() -> {
          driveSub.isFieldRelative = true;
        }));
    driverController.rightStick().onTrue(
        Commands.runOnce(() -> {
          driveSub.isFieldRelative = false;
        }));

    // Indexer commands
    driverController.y().onTrue(new IndexToBeamBreakCmd(indexerSub, () -> 0.75));
    driverController.leftBumper().whileTrue(
        new IndexerSetVelocityManualCmd(indexerSub, () -> -0.75));
    driverController.rightBumper().whileTrue(
        new IndexerSetVelocityManualCmd(indexerSub, () -> 1));

    // Launcher commands
    driverController.leftTrigger().whileTrue(
        new LauncherSetVelocityPIDCmd(launcherSub, () -> -launcherSub.getPreferredFrontVelocity(),
            () -> -launcherSub.getPreferredBackVelocity()));
    driverController.rightTrigger().toggleOnTrue(
        new LauncherSetVelocityPIDCmd(launcherSub, () -> launcherSub.getPreferredFrontVelocity(),
            () -> launcherSub.getPreferredBackVelocity()));
    driverController.povDown().toggleOnTrue(
        new LauncherSetVelocityPIDCmd(launcherSub, () -> LauncherConstants.kVelocityYeetForward,
            () -> LauncherConstants.kVelocityYeetBack));

    // OPERATOR CONTROLLER

    // Arm setpoints
    operatorController.leftTrigger().onTrue(
        new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleStowed));
    operatorController.povDown().onTrue(
        new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleGroundIntake));
    operatorController.povRight().onTrue(
        new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleL2));
    operatorController.povLeft().onTrue(
        new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleL3));
    operatorController.povUp().onTrue(
        new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleProcessor));
    operatorController.rightTrigger().onTrue(
        new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleCoral));

    // Indexer command
    // operatorController.b().onTrue(new IndexToBeamBreakCmd(indexerSub, () ->
    // 0.75));
    operatorController.y().onTrue(Commands.runOnce(() -> {
      launcherSub.setUseHighVelocities(true);
      launcherSub.setReferenceVelocityOffset(0);
    }));
    operatorController.a().onTrue(Commands.runOnce(() -> {
      launcherSub.setUseHighVelocities(false);
      launcherSub.setReferenceVelocityOffset(0);
    }));
    operatorController.x()
        .onTrue(Commands.runOnce(() -> {
          launcherSub.increaseReferenceVelocityOffset(LauncherConstants.kBumpOffsetRPM);
        }));
    operatorController.b()
        .onTrue(Commands.runOnce(() -> {
          launcherSub.increaseReferenceVelocityOffset(-LauncherConstants.kBumpOffsetRPM);
        }));

    // Bump arm setpoints
    operatorController.leftBumper().whileTrue(
        Commands.runOnce(() -> {
          armSub.increaseAngularOffset(-ArmConstants.kBumpOffsetDeg);
        }, armSub));

    operatorController.rightBumper().whileTrue(
        Commands.runOnce(() -> {
          armSub.increaseAngularOffset(ArmConstants.kBumpOffsetDeg);
        }, armSub));
  }

  /**
   * Use this method to define the autonomous command.
   */
  private void configureAutos() {
    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("CalibrateGyro", new CalibrateGyroCmd(driveSub));
    autoChooser.addOption("TimedAutoDrive", new TimedAutoDrive(driveSub));
    autoChooser.addOption("EncoderAutoDrive", new EncoderAutoDrive(driveSub));
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