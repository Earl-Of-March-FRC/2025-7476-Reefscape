// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DriveConstants.LaunchingDistances;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToNearestBargeLaunchingZoneCmd extends Command {
  private final Drivetrain driveSub;
  private final BangBangController translationController = new BangBangController(
      LaunchingDistances.kToleranceMetersFromBarge);
  private final BangBangController rotationController = new BangBangController(
      LaunchingDistances.kToleranceRadiansFromBarge);

  private double targetX, targetRadians;

  private boolean translationFinish = false, rotationFinish = false;

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
    translationFinish = false;
    rotationFinish = false;

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

    // Calculate target pose
    boolean onBlueSide = driveSub.isOnBlueSide();

    // Calculate target translation
    // (0,0) is ALWAYS on the blue alliance side
    targetX = FieldConstants.kBargeX + ((onBlueSide ? -1 : 1) * LaunchingDistances.kMetersFromBarge);

    // Calculate target rotation based on side of field that robot is currently on
    targetRadians = onBlueSide ? Math.PI : 0;

    // For both alliances, 0 rad points away from our driver station
    // if (DriverStation.getAlliance().isPresent()) {
    // Alliance alliance = DriverStation.getAlliance().get();

    // // Check if robot is currently on the same side of field as the alliance
    // // If yes, then target angle is 0 rad; otherwise, pi rad
    // targetRadians = (onBlueSide == (alliance == Alliance.Blue)) ? 0 : Math.PI;
    // } else {
    // targetRadians = currentPose.getRotation().getRadians();
    // }

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/CurrentPose",
        currentPose);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TargetPose",
        new Pose2d(targetX, currentPose.getY(),
            Rotation2d.fromRadians(targetRadians)));

    // Make adjustments to the robot
    double directionX = 0;
    double directionRot = 0;

    // Calculate translation from bang bang controller
    if (!translationFinish) {
      // Bang bang controller returns 0 or 1
      // Multiply calculated output by 2 and subtract 1 to get -1 or 1
      directionX = (translationController.calculate(currentPose.getX(), targetX) * 2) - 1;

      // Reverse direction if on red alliance
      if (DriverStation.getAlliance().isPresent()) {
        Alliance alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Red) {
          directionX *= -1;
        }
      }

      translationFinish = translationController.atSetpoint();
    }

    // Calculate rotation from bang bang controller
    if (!rotationFinish) {
      double currentRotation = currentPose.getRotation().getRadians();

      // Bang bang controller returns 0 or 1
      // Multiply calculated output by 2 and subtract 1 to get -1 or 1
      directionRot = (rotationController.calculate(currentRotation, targetRadians * Math.signum(currentRotation)) * 2)
          - 1;

      rotationFinish = rotationController.atSetpoint();
    }

    // Convert calculated value to velocity
    double xVel = DriveConstants.kBangBangTranslationalVelocityMetersPerSecond * directionX;
    double rotVel = DriveConstants.kBangBangRotationalVelocityRadiansPerSecond * directionRot;

    // Set drivetrain to run at calculated velocity
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