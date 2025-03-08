// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;

/**
 * The IntakeSubsystem class represents the robot's intake subsystem. It manages
 * the intake rollers.
 */
public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax intakeSpark;
  private final RelativeEncoder intakeEncoder;

  /**
   * The constructor for the IntakeSubsystem configures the intake motor.
   */
  public IntakeSubsystem(SparkMax intakeSpark) {
    this.intakeSpark = intakeSpark;

    intakeEncoder = intakeSpark.getEncoder();

    intakeSpark.configure(IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Convert radians per second to Rot per Minute
    Logger.recordOutput("Rollers/Measured/Velocity",
        getVelocity() / IntakeConstants.kVelocityConversionFactor);
    // Logger.recordOutput("Intake/Rollers/Measured/VelocityRad",
    // getVelocity());
  }

  /**
   * Gets the velocity of the intake motor.
   * 
   * @return The current velocity of the intake, in radians per second.
   */
  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  /**
   * Sets the velocity of the intake motor using percent output.
   * 
   * @param percent Percent output, from -1 to 1.
   */
  public void setVelocity(double percent) {
    Logger.recordOutput("Rollers/Setpoint/PercentVelocity", percent);
    intakeSpark.set(percent);
  }

  /**
   * Stops the intake motor.
   */
  public void stopIntake() {
    setVelocity(0);
  }
}
