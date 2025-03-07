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
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
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

  private final ArmSubsystem armSub;
  private final IntakeSubsystem intakeSub;
  private final Indexer indexerSub;
  private final Launcher launcherSub;
  private final AlgaeSubsystem algaeSubsystem;
  public final SwerveDriveSimulation swerveDriveSimulation;

  private final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

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
          new Pose2d(7, 4, new Rotation2d(180)));
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

    intakeSub = new IntakeSubsystem(new SparkMax(IntakeConstants.kMotorCanId, IntakeConstants.kMotorType));

    indexerSub = new Indexer(
        new SparkMax(IndexerConstants.kMotorCanId, IndexerConstants.kMotorType),
        new BeamBreakSensor(IndexerConstants.kIntakeSensorChannel),
        new BeamBreakSensor(IndexerConstants.kLauncherSensorChannel));

    launcherSub = new Launcher(
        new SparkMax(LauncherConstants.kFrontCanId, LauncherConstants.kMotorType),
        new SparkMax(LauncherConstants.kBackCanId, LauncherConstants.kMotorType));

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

    algaeSubsystem = new AlgaeSubsystem(() -> driveSub.getPose());

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
    operatorController.leftTrigger().onTrue(new ArmSetPositionPIDCmd(armSub,
        () -> ArmConstants.kAngleStowed - armSub.armOffset));
    operatorController.povDown().onTrue(
        new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleGroundIntake - armSub.armOffset));
    operatorController.povRight().onTrue(new ArmSetPositionPIDCmd(armSub,
        () -> ArmConstants.kAngleL2 - armSub.armOffset));
    operatorController.povLeft().onTrue(new ArmSetPositionPIDCmd(armSub,
        () -> ArmConstants.kAngleL3 - armSub.armOffset));
    operatorController.povUp().onTrue(new ArmSetPositionPIDCmd(armSub,
        () -> ArmConstants.kAngleProcessor - armSub.armOffset));
    operatorController.rightTrigger().onTrue(new ArmSetPositionPIDCmd(armSub,
        () -> ArmConstants.kAngleCoral - armSub.armOffset));
    operatorController.a()
        .whileTrue(new IntakeSetVelocityManualCmd(intakeSub, () -> IntakeConstants.kDefaultPercent));
    operatorController.y().onTrue(new IndexToBeamBreakCmd(indexerSub, () -> 0.75));
    // operatorController.b().onTrue(new IntakeStopCmd(intakeSub));
    // operatorController.y().onTrue(new ArmResetEncoderCmd(armSub));
    driverController.x().onTrue(new IndexToBeamBreakCmd(indexerSub, () -> -1));
    driverController.y().onTrue(new IndexToBeamBreakCmd(indexerSub, () -> 0.75));
    driverController.rightTrigger().toggleOnTrue(
        new LauncherSetVelocityPIDCmd(launcherSub, () -> launcherSub.getPreferredFrontVelocity(),
            () -> launcherSub.getPreferredBackVelocity()));
    driverController.leftTrigger().whileTrue(
        new LauncherSetVelocityPIDCmd(launcherSub, () -> -launcherSub.getPreferredFrontVelocity(),
            () -> -launcherSub.getPreferredBackVelocity()));
    driverController.rightBumper().whileTrue(
        new IndexerSetVelocityManualCmd(indexerSub, () -> 1));
    driverController.leftBumper().whileTrue(
        new IndexerSetVelocityManualCmd(indexerSub, () -> -1));
    driverController.leftStick().onTrue(
        Commands.runOnce(() -> {
          driveSub.isFieldRelative = true;
        }));
    driverController.rightStick().onTrue(
        Commands.runOnce(() -> {
          driveSub.isFieldRelative = false;
        }));
    operatorController.axisGreaterThan(OIConstants.kOperatorArmManualAxis, OIConstants.kArmDeadband).onTrue(
        Commands.runOnce(() -> {
          armSub.isManual = true;
        }));
    operatorController.axisLessThan(OIConstants.kOperatorArmManualAxis, -OIConstants.kArmDeadband).onTrue(
        Commands.runOnce(() -> {
          armSub.isManual = true;
        }));
    operatorController.leftBumper().whileTrue(
        Commands.runOnce(() -> {
          armSub.armOffset += 2;
        }));

    operatorController.rightBumper().whileTrue(
        Commands.runOnce(() -> {
          armSub.armOffset -= 2;
        }));

    driverController.rightStick().whileTrue(new GoToAlgaeCmd(algaeSubsystem, intakeSub));

    driverController.a().whileTrue(new ConditionalCommand(
        new PathfindToLaunchSpotCmd(AutoConstants.kLaunchPoseBlue),
        new PathfindToLaunchSpotCmd(AutoConstants.kLaunchPoseRed),
        () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue));

    // Arm calibration
    new Trigger(() -> armSub.getLimitSwitch()).onTrue(Commands.runOnce(() -> armSub.resetPosition()));
  }

  /**
   * Use this method to define the autonomous command.
   */
  private void configureAutos() {
    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("TimedAutoDrive", new TimedAutoDrive(driveSub));
    autoChooser.addOption("EncoderAutoDrive", new EncoderAutoDrive(driveSub, 2, 1, 0));
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