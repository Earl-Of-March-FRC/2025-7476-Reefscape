// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;

/**
 * The IntakeSubsystem class represents the robot's intake subsystem. It manages
 * the intake rollers.
 */
public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax intakeSpark = new SparkMax(IntakeConstants.kIntakeMotorCanId, SparkMax.MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeSpark.getEncoder();

  /**
   * The constructor for the IntakeSubsystem configures the intake motor.
   */
  public IntakeSubsystem() {
    intakeSpark.configure(IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Rollers/Measured/Velocity", getIntakeVelocity());
  }

  /**
   * Gets the velocity of the intake motor.
   * 
   * @return
   */
  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  /**
   * Sets the velocity of the intake motor.
   * 
   * @param velocity Desired velocity, from -1 to 1.
   */
  public void setIntakeVelocity(double velocity) {
    Logger.recordOutput("Intake/Rollers/Setpoint/Velocity", velocity);
    intakeSpark.set(velocity);
  }

  /**
   * Stops the intake motor.
   */
  public void stopIntake() {
    setIntakeVelocity(0);
  }
}
