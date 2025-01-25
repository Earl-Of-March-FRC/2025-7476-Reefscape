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
import frc.robot.subsystems.drivetrain.GyroNavX;
import frc.robot.subsystems.drivetrain.MAXSwerveModule;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.SetPipelineCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final VisionSubsystem visionSubsystem;
  public final Gyro gyro;
  public final Drivetrain driveSub;

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // Initialize VisionSubsystem
    this.visionSubsystem = new VisionSubsystem();

    // Create and bind SetPipelineCommand to the Y button press
    SetPipelineCommand setPipelineCommand = new SetPipelineCommand(visionSubsystem, driverController);
    driverController.y().onTrue(setPipelineCommand);

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

    autoChooser = new LoggedDashboardChooser<>("Auto Routine",
        AutoBuilder.buildAutoChooser());
    configureAutos();
    configureBindings();
  }

  private void configureBindings() {
    driverController.b().onTrue(new CalibrateCmd(driveSub));
  }

  /**
   * Use this method to define the autonomous command.
   */
  private void configureAutos() {
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("TimedAutoDrive", new TimedAutoDrive(driveSub));
    SmartDashboard.putData("Auto Routine", autoChooser.getSendableChooser());
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
