// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

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

/**
 * The Drivetrain class represents the robot's drivetrain subsystem.
 * It extends the SubsystemBase class and manages the swerve drive modules and
 * odometry.
 */
public class Drivetrain extends SubsystemBase {
  // Array to hold the four swerve drive modules (front-left, front-right,
  // back-left, back-right)
  private final MAXSwerveModule[] modules = new MAXSwerveModule[4]; // FL, FR, BL, BR

  // Gyro sensor to get the robot's orientation
  private final Gyro gyro;

  // Current pose of the robot
  Pose2d pose;

  // Odometry class for tracking the robot's position on the field
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

  /**
   * Constructor for the Drivetrain class.
   * Initializes the swerve modules and gyro sensor.
   * 
   * @param moduleFL Front-left swerve module
   * @param moduleFR Front-right swerve module
   * @param moduleBL Back-left swerve module
   * @param moduleBR Back-right swerve module
   * @param gyro     Gyro sensor
   */
  public Drivetrain(MAXSwerveModule moduleFL, MAXSwerveModule moduleFR, MAXSwerveModule moduleBL,
      MAXSwerveModule moduleBR, Gyro gyro) {
    modules[0] = moduleFL;
    modules[1] = moduleFR;
    modules[2] = moduleBL;
    modules[3] = moduleBR;
    this.gyro = gyro;
  }

  /**
   * This method is called periodically by the scheduler.
   * It updates the robot's pose using the odometry class.
   */
  @Override
  public void periodic() {
    // Get the current angle from the gyro sensor
    var gyroAngle = gyro.getRotation2d();

    // Update the robot's pose using the odometry class
    pose = odometry.update(gyroAngle,
        new SwerveModulePosition[] {
            modules[0].getPosition(), modules[1].getPosition(),
            modules[2].getPosition(), modules[3].getPosition()
        });

    // Log the current pose to the logger
    Logger.recordOutput("Odometry", pose);

    // Create arrays to hold the states and positions of the swerve modules
    SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // Populate the arrays with the current states and positions of the swerve
    // modules
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
      positions[i] = modules[i].getPosition();
    }

    // Log the states and positions of the swerve modules to the logger
    Logger.recordOutput("Swerve/Module/State", states);
    Logger.recordOutput("Swerve/Module/Position", positions);
  }

  /**
   * Runs the drivetrain at the specified velocities relative to the field.
   * 
   * @param speeds The desired chassis speeds
   */
  public void runVelocityFieldRelative(ChassisSpeeds speeds) {
    runVelocity(speeds, true);
  }

  /**
   * Runs the drivetrain at the specified velocities relative to the robot.
   * 
   * @param speeds The desired chassis speeds
   */
  public void runVelocityRobotRelative(ChassisSpeeds speeds) {
    runVelocity(speeds, false);
  }

  /**
   * Runs the drivetrain at the specified velocities.
   * 
   * @param speeds          The desired chassis speeds
   * @param isFieldRelative Whether the speeds are relative to the field
   */
  public void runVelocity(ChassisSpeeds speeds, Boolean isFieldRelative) {
    // If the speeds are field-relative, convert them to robot-relative speeds
    if (isFieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          gyro.getRotation2d());
    }

    // Convert the chassis speeds to swerve module states
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Desaturate the wheel speeds to ensure they are within the maximum speed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    // Set the desired state for each swerve module
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }

    // Log the desired states of the swerve modules to the logger
    Logger.recordOutput("Swerve/Module/Setpoint", states);
  }

    /**
   * Gets the current state of each swerve module.
   * 
   * @return An array of SwerveModuleState representing the state of each swerve module.
   */
  public SwerveModuleState[] getModuleState() {
      return new SwerveModuleState[] {
          modules[0].getState(),
          modules[1].getState(),
          modules[2].getState(),
          modules[3].getState()
      };
  }
  
  /**
   * Gets the current position of each swerve module.
   * 
   * @return An array of SwerveModulePosition representing the position of each swerve module.
   */
  public SwerveModulePosition[] getModulePositions() {
      return new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      };
  }
  
  /**
   * Gets the current chassis speeds relative to the robot.
   * 
   * @return A ChassisSpeeds object representing the robot-relative chassis speeds.
   */
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
      return DriveConstants.kDriveKinematics.toChassisSpeeds(
          modules[0].getState(),
          modules[1].getState(),
          modules[2].getState(),
          modules[3].getState());
  }
}
