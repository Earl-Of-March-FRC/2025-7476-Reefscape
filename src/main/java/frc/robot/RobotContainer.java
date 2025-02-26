package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CalibrateCmd;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.GoToAlgaeCmd;
import frc.robot.commands.TimedAutoDrive;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.subsystems.drivetrain.GyroADXRS450;
import frc.robot.subsystems.drivetrain.GyroNavX;
import frc.robot.subsystems.drivetrain.MAXSwerveModule;
import frc.robot.subsystems.intake.IntakeSubsystem;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.AlgaeSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.commands.SetPipelineCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

        public final Gyro gyro;
        public final Drivetrain driveSub;
        private final LimelightSubsystem limelightSubsystem;

        private final CommandXboxController driverController = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);
        private final LoggedDashboardChooser<Command> autoChooser;

        private final AlgaeSubsystem algaeSubsystem;
        private final IntakeSubsystem intakeSub;
        private PathPlannerPath algaePath;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                gyro = new GyroNavX();
                gyro.calibrate();

                limelightSubsystem = new LimelightSubsystem();

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

                algaeSubsystem = new AlgaeSubsystem(() -> driveSub.getPose());

                intakeSub = new IntakeSubsystem(new SparkMax(IntakeConstants.kMotorCanId, IntakeConstants.kMotorType));
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

                autoChooser = new LoggedDashboardChooser<>("Auto Routine",
                                AutoBuilder.buildAutoChooser());
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
                driverController.a()
                                .onTrue(new GoToAlgaeCmd(algaeSubsystem, intakeSub))
                                .onFalse(new GoToAlgaeCmd(algaeSubsystem, intakeSub)
                                                .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));

                driverController.b().onTrue(new CalibrateCmd(driveSub));

                // Bind the Y button to the SetPipelineCommand
                driverController.y().onTrue(new SetPipelineCommand(limelightSubsystem, driverController.y()));
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
}