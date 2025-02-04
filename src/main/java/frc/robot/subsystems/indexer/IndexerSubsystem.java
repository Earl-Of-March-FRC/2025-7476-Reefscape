// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax indexerSpark;
  private final DigitalInput intakeSensor, shooterSensor;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;

  private final SparkSim indexerSparkSim; // In case we ever need to simulate the motor

  /**
   * Creates a new IndexerSubsystem
   * 
   * @param motorPort            CAN ID of the indexer motor
   * @param motorType            MotorType of the indexer motor
   * @param intakeSensorChannel  DIO port of the digital sensor near the intake
   * @param shooterSensorChannel DIO port of the digital sensor near the shooter
   */
  public IndexerSubsystem(int motorPort, MotorType motorType, int intakeSensorChannel, int shooterSensorChannel) {
    indexerSpark = new SparkMax(motorPort, motorType);
    intakeSensor = new DigitalInput(intakeSensorChannel);
    shooterSensor = new DigitalInput(shooterSensorChannel);

    indexerSparkSim = new SparkSim(indexerSpark, DCMotor.getNeo550(1));

    indexerSpark.configure(Configs.IndexerSubsystem.indexerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    controller = indexerSpark.getClosedLoopController();
    encoder = indexerSpark.getEncoder();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("indexerIntakeSensor", getIntakeSensor());
    Logger.recordOutput("indexerShooterSensor", getShooterSensor());
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Set the output speed of the motors.
   * 
   * @param percent Percentage between -1.0 and +1.0
   * @see #setVoltage
   */
  public void setSpeed(double percent) {
    indexerSpark.set(percent * IndexerConstants.kDirectionConstant);
  };

  /**
   * Get the output speed of the motors.
   * 
   * @return Percent between -1.0 and +1.0
   * @see #getVoltage
   */
  public double getSpeed() {
    return indexerSpark.get();
  }

  /**
   * Set the output voltage of the motors.
   * 
   * @param voltage Output voltage.
   * @see #setSpeed
   */
  public void setVoltage(double voltage) {
    indexerSpark.setVoltage(voltage * IndexerConstants.kDirectionConstant);
  };

  /**
   * Get the output voltage of the motors.
   * 
   * @return Output voltage.
   * @see #getSpeed
   */
  public double getVoltage() {
    return indexerSpark.getAppliedOutput();
  }

  /**
   * Set the velocity (RPM) of the indexer motors in meters
   * 
   * @param velocity RPM in meters
   */
  public void setVelocity(double velocity) {
    controller.setReference(velocity, ControlType.kVelocity);
  }

  /**
   * Get the velocity (RPM) of the indexer motors in meters.
   * 
   * @return RPM in meters
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Get the value of the sensor near the intake.
   * 
   * @return {@code true} if sensor is tripped or not connected, {@code false} if
   *         not tripped.
   * @see #getShooterSensor
   */
  public boolean getIntakeSensor() {
    return intakeSensor.get();
  }

  /**
   * Get the value of the sensor near the shooter.
   * 
   * @return {@code true} if sensor is tripped or not connected, {@code false} if
   *         not tripped.
   * @see #getIntakeSensor
   */
  public boolean getShooterSensor() {
    return shooterSensor.get();
  }
}
