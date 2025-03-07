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

  private double frontReferenceVelocity = 0.0;
  private double backReferenveVelocity = 0.0;

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

    SmartDashboard.putNumber("LauncherFrontVelocity", LauncherConstants.kVelocityFront);
    SmartDashboard.putNumber("LauncherBackVelocity", LauncherConstants.kVelocityBack);
  }

  /**
   * This method is called periodically by the scheduler.
   * It logs the launcher velocity in RPM.
   */
  @Override
  public void periodic() {
    // Converts radians per second to RPM
    Logger.recordOutput("Launcher/Front/Measured/Velocity",
        getFrontVelocity() / LauncherConstants.kVelocityConversionFactor);
    Logger.recordOutput("Launcher/Back/Measured/Velocity",
        getBackVelocity() / LauncherConstants.kVelocityConversionFactor);
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

    SmartDashboard.putBoolean("LauncherFrontAtSetpoint", frontRollerAtSetpoint());
    SmartDashboard.putBoolean("LauncherBackAtSetpoint", backRollerAtSetpoint());

    frontLauncherSpark.set(percent);
    backLauncherSpark.set(percent);
  }

  /**
   * Sets the reference velocity for both launcher closed loop controllers.
   * 
   * @param referenceVelocity The reference velocity, in RPM.
   */
  public void setReferenceVelocity(double referenceVelocity) {
    setFrontReferenceVelocity(referenceVelocity);
    setBackReferenceVelocity(referenceVelocity);
  }

  /**
   * Sets the reference velocity for the front launcher closed loop controller.
   *
   * @param referenceVelocity The reference velocity, in RPM.
   */
  public void setFrontReferenceVelocity(double referenceVelocity) {
    frontReferenceVelocity = referenceVelocity;
    Logger.recordOutput("Launcher/Front/Setpoint/Velocity", referenceVelocity);

    // Converts RPM to radians per second
    frontLauncherClosedLoopController.setReference(referenceVelocity * LauncherConstants.kVelocityConversionFactor,
        ControlType.kVelocity);
  }

  /**
   * Sets the reference velocity for the back launcher closed loop controller.
   *
   * @param referenceVelocity The reference velocity, in RPM.
   */
  public void setBackReferenceVelocity(double referenceVelocity) {
    backReferenveVelocity = referenceVelocity;
    Logger.recordOutput("Launcher/Back/Setpoint/Velocity", referenceVelocity);

    // Converts RPM to radians per second
    backLauncherClosedLoopController.setReference(referenceVelocity * LauncherConstants.kVelocityConversionFactor,
        ControlType.kVelocity);
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
    return SmartDashboard.getNumber("LauncherFrontVelocity", LauncherConstants.kVelocityFront);
  }

  /**
   * Get the preferred reference velocity for the back rollers.
   * 
   * @return Preferred velocity
   */
  public double getPreferredBackVelocity() {
    return SmartDashboard.getNumber("LauncherBackVelocity", LauncherConstants.kVelocityBack);
  }

  public boolean frontRollerAtSetpoint() {
    return MathUtil.isNear(frontReferenceVelocity, getFrontVelocity() / LauncherConstants.kVelocityConversionFactor,
        LauncherConstants.kVelocityFrontTolerance);
  }

  public boolean backRollerAtSetpoint() {
    return MathUtil.isNear(backReferenveVelocity, getBackVelocity() / LauncherConstants.kVelocityConversionFactor,
        LauncherConstants.kVelocityBackTolerance);
  }
}
