package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.TimedAutoDrive;
import frc.robot.commands.Launcher.LauncherSetVelocityPIDCmd;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.subsystems.drivetrain.GyroNavX;
import frc.robot.subsystems.drivetrain.MAXSwerveModule;

public class RobotContainer {

        public final Drivetrain driveSub;
        public final Gyro gyro;

        private final CommandXboxController driverController = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);

        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");;
        final LauncherSubsystem shooter = new LauncherSubsystem();
        private final CommandXboxController controller = new CommandXboxController(0);

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

        private void configureBindings() {
                controller.rightTrigger().onTrue(new LauncherSetVelocityPIDCmd(shooter, () -> 60.0));
                controller.rightBumper().onTrue(new LauncherSetVelocityPIDCmd(shooter, () -> 200.0));
                controller.leftTrigger().onTrue(new LauncherSetVelocityPIDCmd(shooter, () -> 0.0));
                controller.leftBumper().onTrue(new LauncherSetVelocityPIDCmd(shooter, () -> 100.0));
        }

        private void configureAutos() {
                autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
                autoChooser.addOption("TimedAutoDrive", new TimedAutoDrive(driveSub));
                SmartDashboard.putData("Auto Routine", autoChooser.getSendableChooser());
        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        // Optional: Add a periodic method to update SmartDashboard values dynamically
        public void updateSmartDashboard() {
                double currentVelocity = shooter.getFrontVelocity(); // Assuming you have a method to get the current
                                                                     // shooter speed
                SmartDashboard.putNumber("CurrentLauncherSpeed", currentVelocity);
                System.out.println("Current Launcher Speed: " + currentVelocity);
        }
}