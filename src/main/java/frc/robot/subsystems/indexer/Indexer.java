// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IndexerConfigs;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.indexer.sensors.IndexerSensor;

public class Indexer extends SubsystemBase {
  private final SparkMax indexerSpark;
  private final RelativeEncoder encoder;

  public final IndexerSensor intakeSensor, launcherSensor;

  /**
   * Creates a new IndexerSubsystem
   * 
   * @param indexerSpark   SparkMax motor controller for the indexer
   * @param intakeSensor   Sensor near the intake
   * @param launcherSensor Sensor near the launcher
   */
  public Indexer(SparkMax indexerSpark, IndexerSensor intakeSensor, IndexerSensor launcherSensor) {
    this.intakeSensor = intakeSensor;
    this.launcherSensor = launcherSensor;
    this.indexerSpark = indexerSpark;

    indexerSpark.configure(IndexerConfigs.indexerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    encoder = indexerSpark.getEncoder();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/Measured/IntakeSensor", getIntakeSensor());
    Logger.recordOutput("Indexer/Measured/LauncherSensor", getLauncherSensor());
    Logger.recordOutput("Indexer/Measured/Velocity", getVelocity());
    Logger.recordOutput("Indexer/Measured/Voltage", getVoltage());
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Set the output velocity of the motors.
   * 
   * @param percent Percentage between -1.0 and +1.0
   * @see #setVoltage
   */
  public void setVelocity(double percent) {
    Logger.recordOutput("Indexer/Setpoint/PercentVelocity", percent);
    indexerSpark.set(percent * IndexerConstants.kDirectionConstant);
  };

  /**
   * Get the velocity of the indexer belts in meters.
   * 
   * @return velocity of belts in meters
   * @see #getVoltage
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Set the output voltage of the motors.
   * 
   * @param voltage Output voltage.
   * @see #setVelocity
   */
  public void setVoltage(double voltage) {
    Logger.recordOutput("Indexer/Setpoint/Voltage", voltage);
    indexerSpark.setVoltage(voltage * IndexerConstants.kDirectionConstant);
  };

  /**
   * Get the output voltage of the motors.
   * 
   * @return Output voltage.
   * @see #getVelocity
   */
  public double getVoltage() {
    return indexerSpark.getAppliedOutput();
  }

  /**
   * Get the value of the sensor near the intake.
   * 
   * @return {@code true} if sensor is tripped or not connected, {@code false} if
   *         not tripped.
   * @see #getLauncherSensor
   */
  public boolean getIntakeSensor() {
    return intakeSensor.triggered();
  }

  /**
   * Get the value of the sensor near the launcher.
   * 
   * @return {@code true} if sensor is tripped or not connected, {@code false} if
   *         not tripped.
   * @see #getIntakeSensor
   */
  public boolean getLauncherSensor() {
    return launcherSensor.triggered();
  }
}
