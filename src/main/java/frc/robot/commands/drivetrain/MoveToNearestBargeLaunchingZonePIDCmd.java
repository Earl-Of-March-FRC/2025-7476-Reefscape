// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DriveConstants.LaunchingDistances;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.PoseHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToNearestBargeLaunchingZonePIDCmd extends Command {
  private final Drivetrain driveSub;
  private final PIDController translationController = new PIDController(AutoConstants.kPTranslationController,
      AutoConstants.kITranslationController, AutoConstants.kDTranslationController);
  private final PIDController rotationController = new PIDController(AutoConstants.kPThetaController,
      AutoConstants.kIThetaController, AutoConstants.kDThetaController);

  private double targetX, targetRadians;

  /** Creates a new MoveToNearestBargeLaunchingZoneCmd. */
  public MoveToNearestBargeLaunchingZonePIDCmd(
      Drivetrain driveSub) {
    this.driveSub = driveSub;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translationController.setTolerance(LaunchingDistances.kToleranceMetersFromBarge);
    rotationController.setTolerance(LaunchingDistances.kToleranceRadiansFromBarge);
  }

  @Override
  public void execute() {
    Pose2d currentPose = driveSub.getPose();

    // Calculate target pose
    boolean onBlueSide = PoseHelpers.isOnBlueSide(currentPose);

    // Calculate target translation
    // (0,0) is ALWAYS on the blue alliance side
    targetX = FieldConstants.kBargeX + ((onBlueSide ? -1 : 1) * LaunchingDistances.kMetersFromBarge);

    // Calculate target rotation based on side of field that robot is currently on
    targetRadians = onBlueSide ? Math.PI : 0;

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/CurrentPose",
        currentPose);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TargetPose",
        new Pose2d(targetX, currentPose.getY(),
            Rotation2d.fromRadians(targetRadians)));

    double xVel = MathUtil.clamp(translationController.calculate(currentPose.getX(), targetX),
        -AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxSpeedMetersPerSecond);

    // Invert the direction if robot is on red alliance
    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Red) {
        xVel *= -1;
      }
    }

    double currentRotation = currentPose.getRotation().getRadians();

    double rotVel = MathUtil.clamp(
        rotationController.calculate(currentRotation, targetRadians * Math.signum(currentRotation)),
        -AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecond);

    // Set drivetrain to run at calculated velocity
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel, 0, rotVel);
    driveSub.runVelocityFieldRelative(chassisSpeeds);

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/PID/OutputVelocityX", xVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/PID/OutputVelocityRotation", rotVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/PID/OutputChassisSpeeds", chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}