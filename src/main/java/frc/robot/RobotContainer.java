// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.arm.ArmSetPositionPIDCmd;
import frc.robot.commands.arm.ArmSetVelocityManualCmd;
import frc.robot.commands.indexer.IndexToBeamBreakCmd;
import frc.robot.commands.indexer.IndexerSetVelocityManualCmd;
import frc.robot.commands.intake.IntakeSetVelocityManualCmd;
import frc.robot.commands.launcher.LauncherSetVelocityPIDCmd;
import frc.robot.commands.vision.GoToAlgaeCmd;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.subsystems.drivetrain.GyroNavX;
import frc.robot.subsystems.drivetrain.GyroSim;
import frc.robot.subsystems.drivetrain.MAXSwerveModule;
import frc.robot.subsystems.drivetrain.SwerveModuleSim;
import frc.robot.subsystems.indexer.BeamBreakSensor;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.vision.AlgaeSubsystem;

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
  private final AlgaeSubsystem algaeSubsystem;
  public final SwerveDriveSimulation swerveDriveSimulation;

  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  // Register Named Commands

  private LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (RobotBase.isReal()) {
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

      swerveDriveSimulation = null;
    } else {

      DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofGenericGyro())
          .withSwerveModule(COTS.ofMAXSwerve(
              DCMotor.getNEO(1),
              DCMotor.getNeo550(1),
              COTS.WHEELS.COLSONS.cof,
              2))
          .withTrackLengthTrackWidth(
              Units.Meters.of(DriveConstants.kWheelBase),
              Units.Meters.of(DriveConstants.kTrackWidth))
          .withBumperSize(Units.Meters.of(0.75), Units.Meters.of(0.75));
      swerveDriveSimulation = new SwerveDriveSimulation(
          driveTrainSimulationConfig,
          new Pose2d(7, 4, Rotation2d.fromDegrees(180)));
      gyro = new GyroSim(swerveDriveSimulation.getGyroSimulation());

      driveSub = new Drivetrain(
          new SwerveModuleSim(swerveDriveSimulation.getModules()[0]),
          new SwerveModuleSim(swerveDriveSimulation.getModules()[1]),
          new SwerveModuleSim(swerveDriveSimulation.getModules()[2]),
          new SwerveModuleSim(swerveDriveSimulation.getModules()[3]),
          gyro, swerveDriveSimulation);

      SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
    }

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

    algaeSubsystem = new AlgaeSubsystem(() -> driveSub.getPose());

    // Register named Commands
    NamedCommands.registerCommand("Calibrate", new CalibrateGyroCmd(driveSub));

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
            () -> MathUtil.applyDeadband(
                operatorController.getRawAxis(
                    OIConstants.kOperatorIntakeManualAxis),
                OIConstants.kIntakeDeadband)));

    // DRIVER CONTROLLER

    // Drive commands
    driverController.b().onTrue(new CalibrateGyroCmd(driveSub));
    driverController.a().whileTrue(new MoveToNearestBargeLaunchingZoneBangBangCmd(driveSub));
    driverController.x().whileTrue(new MoveToNearestBargeLaunchingZonePIDCmd(driveSub));

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
        new IndexerSetVelocityManualCmd(indexerSub, () -> -1));
    driverController.rightBumper().whileTrue(
        new IndexerSetVelocityManualCmd(indexerSub, () -> 1));

    // Launcher commands
    driverController.leftTrigger().whileTrue(
        new LauncherSetVelocityPIDCmd(launcherSub, () -> -launcherSub.getPreferredFrontVelocity(),
            () -> -launcherSub.getPreferredBackVelocity()));
    driverController.rightTrigger().toggleOnTrue(
        new LauncherSetVelocityPIDCmd(launcherSub, () -> launcherSub.getPreferredFrontVelocity(),
            () -> launcherSub.getPreferredBackVelocity()));

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
    operatorController.b().onTrue(new IndexToBeamBreakCmd(indexerSub, () -> 0.75));

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