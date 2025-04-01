// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.LauncherConfigs;
import frc.robot.Constants.LauncherConstants;

/**
 * LauncherSubsystem controls the launcher mechanism of the robot.
 * It manages motor speed, velocity, and PID control for the launcher.
 */
public class Launcher extends SubsystemBase {
  private final SparkMax frontLauncherSpark;
  private final RelativeEncoder frontLauncherEncoder;
  private final SparkClosedLoopController frontLauncherClosedLoopController;

  private final SparkMax backLauncherSpark;
  private final RelativeEncoder backLauncherEncoder;
  private final SparkClosedLoopController backLauncherClosedLoopController;

  private double frontReferenceVelocityWithoutOffset = 0.0;
  private double backReferenceVelocityWithoutOffset = 0.0;
  private double velocityOffsetRPM = 0.0;

  private boolean useHighVelocities = true;

  /**
   * Constructs a new LauncherSubsystem and configures the launcher motors.
   */
  public Launcher(SparkMax frontLauncherSpark, SparkMax backLauncherSpark) {
    this.frontLauncherSpark = frontLauncherSpark;
    this.backLauncherSpark = backLauncherSpark;

    this.frontLauncherEncoder = frontLauncherSpark.getEncoder();
    this.backLauncherEncoder = backLauncherSpark.getEncoder();
    this.frontLauncherClosedLoopController = frontLauncherSpark.getClosedLoopController();
    this.backLauncherClosedLoopController = backLauncherSpark.getClosedLoopController();

    // Configure motors
    frontLauncherSpark.configure(LauncherConfigs.frontLauncherConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    backLauncherSpark.configure(LauncherConfigs.backLauncherConfig.inverted(true), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Log in rad/s
    SmartDashboard.putNumber("LauncherHighFrontVelocity",
        LauncherConstants.kVelocityHighFrontRPM * LauncherConstants.kVelocityConversionFactor);
    SmartDashboard.putNumber("LauncherHighBackVelocity",
        LauncherConstants.kVelocityHighBackRPM * LauncherConstants.kVelocityConversionFactor);
    SmartDashboard.putNumber("LauncherLowFrontVelocity",
        LauncherConstants.kVelocityLowFrontRPM * LauncherConstants.kVelocityConversionFactor);
    SmartDashboard.putNumber("LauncherLowBackVelocity",
        LauncherConstants.kVelocityLowBackRPM * LauncherConstants.kVelocityConversionFactor);

  }

  /**
   * This method is called periodically by the scheduler.
   * It logs the launcher velocity in RPM.
   */
  @Override
  public void periodic() {
    // Log in rad/s
    Logger.recordOutput("Launcher/Front/Measured/Velocity", getFrontVelocity());
    Logger.recordOutput("Launcher/Back/Measured/Velocity", getBackVelocity());
    SmartDashboard.putBoolean("LauncherFrontAtSetpoint", frontRollerAtSetpoint());
    SmartDashboard.putBoolean("LauncherBackAtSetpoint", backRollerAtSetpoint());
    SmartDashboard.putNumber("FrontVel", getFrontVelocity());
    SmartDashboard.putNumber("BackVel", getBackVelocity());
  }

  /**
   * Gets the velocity of the front launcher motor.
   *
   * @return The velocity of the front launcher motor, in radians per second.
   */
  public double getFrontVelocity() {
    return frontLauncherEncoder.getVelocity();
  }

  /**
   * Gets the velocity of the back launcher motor.
   *
   * @return The velocity of the back launcher motor, in radians per second.
   */
  public double getBackVelocity() {
    return backLauncherEncoder.getVelocity();
  }

  /**
   * Sets the velocity of both launcher motors using percent output.
   *
   * @param percent The percent output, from -1 to 1.
   */
  public void setVelocity(double percent) {
    Logger.recordOutput("Launcher/Front/Setpoint/PercentVelocity", percent);
    Logger.recordOutput("Launcher/Back/Setpoint/PercentVelocity", percent);

    frontLauncherSpark.set(percent);
    backLauncherSpark.set(percent);
  }

  /**
   * Sets the reference velocity for both launcher closed loop controllers.
   * 
   * @param referenceVelocity The reference velocity, in RPM.
   */
  public void setReferenceVelocity(double referenceVelocity) {
    setReferenceVelocity(referenceVelocity, referenceVelocity);
  }

  /**
   * Sets the reference velocity for both launcher closed loop controllers.
   * 
   * @param frontReferenceVelocity The front reference velocity, in RPM.
   * @param backReferenceVelocity  The back reference velocity, in RPM.
   */
  public void setReferenceVelocity(double frontReferenceVelocity, double backReferenceVelocity) {
    setFrontReferenceVelocity(frontReferenceVelocity);
    setBackReferenceVelocity(backReferenceVelocity);
  }

  /**
   * Override the velocity offset
   * 
   * @param offsetRPM New offset in RPM
   */
  public void setReferenceVelocityOffset(double offsetRPM) {
    velocityOffsetRPM = offsetRPM;
    Logger.recordOutput("Launcher/VelocityOffsetRPM", velocityOffsetRPM);
    Logger.recordOutput("Launcher/VelocityOffsetRadPerSec",
        velocityOffsetRPM * LauncherConstants.kVelocityConversionFactor);

    setReferenceVelocity(frontReferenceVelocityWithoutOffset, backReferenceVelocityWithoutOffset);
  }

  /**
   * Increase the velocity offset
   * 
   * @param offsetRPM RPM to increase by
   */
  public void increaseReferenceVelocityOffset(double offsetRPM) {
    setReferenceVelocityOffset(velocityOffsetRPM + offsetRPM);
  }

  /**
   * Sets the reference velocity for the front launcher closed loop controller.
   *
   * @param referenceVelocity The reference velocity, in RPM.
   */
  public void setFrontReferenceVelocity(double referenceVelocity) {
    double referenceVelocityWithOffset = referenceVelocity + velocityOffsetRPM;
    frontReferenceVelocityWithoutOffset = referenceVelocity;
    Logger.recordOutput("Launcher/Front/Setpoint/Velocity", referenceVelocity);
    Logger.recordOutput("Launcher/Front/Setpoint/VelocityWithOffset", referenceVelocityWithOffset);

    // Converts RPM to radians per second
    frontLauncherClosedLoopController.setReference(
        referenceVelocityWithOffset * LauncherConstants.kVelocityConversionFactor,
        ControlType.kVelocity, isUsingHighVelocities() ? LauncherConstants.kSlotHigh : LauncherConstants.kSlotLow);
  }

  /**
   * Sets the reference velocity for the back launcher closed loop controller.
   *
   * @param referenceVelocity The reference velocity, in RPM.
   */
  public void setBackReferenceVelocity(double referenceVelocity) {
    double referenceVelocityWithOffset = referenceVelocity + velocityOffsetRPM;
    backReferenceVelocityWithoutOffset = referenceVelocity;
    Logger.recordOutput("Launcher/Back/Setpoint/Velocity", referenceVelocity);
    Logger.recordOutput("Launcher/Back/Setpoint/VelocityWithOffset", referenceVelocityWithOffset);

    // Converts RPM to radians per second
    backLauncherClosedLoopController.setReference(
        referenceVelocityWithOffset * LauncherConstants.kVelocityConversionFactor,
        ControlType.kVelocity, isUsingHighVelocities() ? LauncherConstants.kSlotHigh : LauncherConstants.kSlotLow);
  }

  /**
   * Stops the launcher motors.
   */
  public void stopLauncher() {
    setReferenceVelocity(0);
  }

  /**
   * Get the preferred reference velocity for the front rollers.
   * 
   * @return Preferred velocity
   */
  public double getPreferredFrontVelocity() {
    if (useHighVelocities) {
      return SmartDashboard.getNumber("LauncherHighFrontVelocity",
          LauncherConstants.kVelocityHighFrontRPM * LauncherConstants.kVelocityConversionFactor)
          / LauncherConstants.kVelocityConversionFactor;
    }
    return SmartDashboard.getNumber("LauncherLowFrontVelocity",
        LauncherConstants.kVelocityLowFrontRPM * LauncherConstants.kVelocityConversionFactor)
        / LauncherConstants.kVelocityConversionFactor;
  }

  /**
   * Get the preferred reference velocity for the back rollers.
   * 
   * @return Preferred velocity
   */
  public double getPreferredBackVelocity() {
    if (useHighVelocities) {
      return SmartDashboard.getNumber("LauncherHighBackVelocity",
          LauncherConstants.kVelocityHighBackRPM * LauncherConstants.kVelocityConversionFactor)
          / LauncherConstants.kVelocityConversionFactor;
    }
    return SmartDashboard.getNumber("LauncherLowBackVelocity",
        LauncherConstants.kVelocityLowBackRPM * LauncherConstants.kVelocityConversionFactor)
        / LauncherConstants.kVelocityConversionFactor;
  }

  /**
   * Set whether the preferred velocities should be the high velocities or low
   * velocities.
   * 
   * @param use {@code true} if it should use the high velocities, {@false} if
   *            it should use the low velocities
   */
  public void setUseHighVelocities(boolean use) {
    useHighVelocities = use;
    Logger.recordOutput("Launcher/UseHighVelocities", useHighVelocities);
  }

  /**
   * Check if the launcher is using high velocities
   * 
   * @return {@code true} if it is using high velocities, {@code false} if it is
   *         using low velocities
   */
  public boolean isUsingHighVelocities() {
    return useHighVelocities;
  }

  /**
   * Check if the front roller has reached the setpoint
   */
  public boolean frontRollerAtSetpoint() {
    return MathUtil.isNear(frontReferenceVelocityWithoutOffset + velocityOffsetRPM,
        getFrontVelocity() / LauncherConstants.kVelocityConversionFactor,
        LauncherConstants.kVelocityFrontTolerance);
  }

  /**
   * Check if the back roller has reached the setpoint
   */
  public boolean backRollerAtSetpoint() {
    return MathUtil.isNear(backReferenceVelocityWithoutOffset + velocityOffsetRPM,
        getBackVelocity() / LauncherConstants.kVelocityConversionFactor,
        LauncherConstants.kVelocityBackTolerance);
  }
}
