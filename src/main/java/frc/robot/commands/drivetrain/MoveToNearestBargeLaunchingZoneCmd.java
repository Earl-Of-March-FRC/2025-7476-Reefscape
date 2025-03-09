// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.utils.Wrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToNearestBargeLaunchingZoneCmd extends Command {
  private final Drivetrain driveSub;
  private double translationDirection, rotationDirection,
      targetX, targetRadians;

  private boolean onSameSide;

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
    translationDirection = Math.signum(targetX - startingPose.getX());

    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      onSameSide = (onBlueSide == (alliance == Alliance.Blue));
      targetRadians = onSameSide ? 0 : Math.PI;
    } else {
      targetRadians = startingPose.getRotation().getRadians();
      onSameSide = true;
    }
    rotationDirection = Math
        .signum(Wrapper.wrapRadian(
            targetRadians - Wrapper.wrapRadian(startingPose.getRotation().getRadians())));

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/StartingPose", startingPose);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TargetPose",
        new Pose2d(targetX, startingPose.getY(), Rotation2d.fromRadians(targetRadians)));

  }

  @Override
  public void execute() {
    double xVel = 0, oVel = 0;
    Pose2d currentPose = driveSub.getPose();
    double currentRadian = Wrapper.wrapRadian(currentPose.getRotation().getRadians());
    if ((translationDirection > 0) ? (currentPose.getX() < targetX) : (currentPose.getX() > targetX)) {
      xVel = translationDirection * DriveConstants.kBangBangTranslationalVelocityMetersPerSecond;
    }
    if (onSameSide) {
      if ((rotationDirection > 0) ? (currentRadian < targetRadians) : (currentRadian > targetRadians)) {
        oVel = rotationDirection * DriveConstants.kBangBangRotationalVelocityRadiansPerSecond;
      }
    } else {
      if ((rotationDirection > 0) ? (currentRadian >= 0) : (currentRadian <= 0)) {
        oVel = rotationDirection * DriveConstants.kBangBangRotationalVelocityRadiansPerSecond;
      }
    }

    driveSub.runVelocityFieldRelative(new ChassisSpeeds(xVel, 0, oVel));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
