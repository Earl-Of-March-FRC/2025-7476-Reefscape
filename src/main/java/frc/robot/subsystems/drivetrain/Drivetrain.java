// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private final MAXSwerveModule[] modules = new MAXSwerveModule[4]; // FL, FR, BL, BR
  private final Gyro gyro;
  Pose2d pose;

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      new Rotation2d(),
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      },
      new Pose2d(0, 0, new Rotation2d()));

  public Drivetrain(MAXSwerveModule moduleFL, MAXSwerveModule moduleFR, MAXSwerveModule moduleBL,
      MAXSwerveModule moduleBR, Gyro gyro) {
    modules[0] = moduleFL;
    modules[1] = moduleFR;
    modules[2] = moduleBL;
    modules[3] = moduleBR;
    this.gyro = gyro;
  }

  @Override
  public void periodic() {
    var gyroAngle = gyro.getRotation2d();
    pose = odometry.update(gyroAngle,
        new SwerveModulePosition[] {
            modules[0].getPosition(), modules[1].getPosition(),
            modules[2].getPosition(), modules[3].getPosition()
        });
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
  }

  public void runVelocityFieldRelative(ChassisSpeeds speeds) {
    runVelocity(speeds, true);
  }

  public void runVelocityRobotRelative(ChassisSpeeds speeds) {
    runVelocity(speeds, false);
  }

  public void runVelocity(ChassisSpeeds speeds, Boolean isFieldRelative) {
    if (isFieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          gyro.getRotation2d());
    }
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  public SwerveModuleState[] getModuleState() {
    return new SwerveModuleState[] {
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    };
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState());
  }
}
