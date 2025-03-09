// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DriveConstants.LaunchingDistances;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Wrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToNearestBargeLaunchingZoneCmd extends Command {
  private final Drivetrain driveSub;
  private final BangBangController translationController = new BangBangController(
      LaunchingDistances.kToleranceMetersFromBarge);
  private final BangBangController rotationController = new BangBangController(
      LaunchingDistances.kToleranceRadiansFromBarge);

  private double targetX, targetRadians;

  // private boolean translationFinish = false, rotationFinish = false;

  // private double translationDirection, rotationDirection,
  // targetX, targetRadians;

  // private final Timer timer = new Timer();

  // private boolean onSameSide;

  /** Creates a new MoveToNearestBargeLaunchingZoneCmd. */
  public MoveToNearestBargeLaunchingZoneCmd(
      Drivetrain driveSub) {
    this.driveSub = driveSub;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d startingPose = driveSub.getPose();
    boolean onBlueSide = driveSub.isOnBlueSide();

    targetX = FieldConstants.kBargeX + ((onBlueSide ? -1 : 1) * LaunchingDistances.kMetersFromBarge);

    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Blue) {
        targetRadians = (onBlueSide == (alliance == Alliance.Blue)) ? Math.PI : 0;
      } else {
        targetRadians = (onBlueSide == (alliance == Alliance.Blue)) ? 0 : Math.PI;
      }
    } else {
      targetRadians = startingPose.getRotation().getRadians();
    }

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/StartingPose",
        startingPose);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TargetPose",
        new Pose2d(targetX, startingPose.getY(),
            Rotation2d.fromRadians(targetRadians)));

    // timer.restart();

    // Pose2d startingPose = driveSub.getPose();
    // boolean onBlueSide = driveSub.isOnBlueSide();

    // targetX = FieldConstants.kBargeX + ((onBlueSide ? -1 : 1) *
    // LaunchingDistances.kMetersFromBarge);
    // translationDirection = Math.signum(targetX - startingPose.getX());

    // if (DriverStation.getAlliance().isPresent()) {
    // Alliance alliance = DriverStation.getAlliance().get();
    // onSameSide = (onBlueSide == (alliance == Alliance.Blue));
    // targetRadians = onSameSide ? 0 : Math.PI;
    // } else {
    // targetRadians = startingPose.getRotation().getRadians();
    // onSameSide = true;
    // }
    // rotationDirection = Math
    // .signum(Wrapper.wrapRadian(
    // targetRadians -
    // Wrapper.wrapRadian(startingPose.getRotation().getRadians())));

    // Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/StartingPose",
    // startingPose);
    // Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TargetPose",
    // new Pose2d(targetX, startingPose.getY(),
    // Rotation2d.fromRadians(targetRadians)));
    // Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TranslationDirection",
    // translationDirection);
    // Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/RotationDirection",
    // rotationDirection);
  }

  @Override
  public void execute() {
    Pose2d currentPose = driveSub.getPose();

    double directionX = 0;
    double directionRot = 0;

    // if (!translationFinish) {
    // translationFinish = translationController.atSetpoint();
    directionX = (translationController.calculate(currentPose.getX(), targetX) * 2) - 1;
    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Red) {
        directionX *= -1;
      }
    }

    // }
    // if (!rotationFinish) {
    double currentRotation = currentPose.getRotation().getRadians();
    // rotationFinish = rotationController.atSetpoint();
    directionRot = (rotationController.calculate(currentRotation, targetRadians * Math.signum(currentRotation)) * 2)
        - 1;
    // }

    double xVel = DriveConstants.kBangBangTranslationalVelocityMetersPerSecond * directionX;
    double rotVel = DriveConstants.kBangBangRotationalVelocityRadiansPerSecond * directionRot;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel, 0, rotVel);

    driveSub.runVelocityFieldRelative(chassisSpeeds);

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/OutputDirectionX", directionX);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/OutputDirectionRotation", directionRot);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/OutputVelocityX", xVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/OutputVelocityRotation", rotVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/OutputChassisSpeeds", chassisSpeeds);

    // double xVel = 0, oVel = 0;
    // Pose2d currentPose = driveSub.getPose();
    // double currentRadian =
    // Wrapper.wrapRadian(currentPose.getRotation().getRadians());
    // if ((translationDirection > 0) ? (currentPose.getX() < targetX) :
    // (currentPose.getX() > targetX)) {
    // xVel = translationDirection *
    // DriveConstants.kBangBangTranslationalVelocityMetersPerSecond;
    // }
    // if (onSameSide) {
    // if ((rotationDirection > 0) ? (currentRadian < targetRadians) :
    // (currentRadian > targetRadians)) {
    // oVel = rotationDirection *
    // DriveConstants.kBangBangRotationalVelocityRadiansPerSecond;
    // }
    // } else {
    // if ((rotationDirection > 0) ? (currentRadian >= 0) : (currentRadian <= 0)) {
    // oVel = rotationDirection *
    // DriveConstants.kBangBangRotationalVelocityRadiansPerSecond;
    // }
    // }

    // driveSub.runVelocityFieldRelative(new ChassisSpeeds(xVel, 0, oVel));
  }

  @Override
  public void end(boolean interrupted) {
    // timer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
    // return translationFinish && rotationFinish;
  }
}
