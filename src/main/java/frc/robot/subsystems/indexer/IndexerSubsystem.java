// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private final SparkMax indexerSpark;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;

  public final DigitalOutput intakeSensorTrigger, launcherSensorTrigger;
  public final IndexerSensor intakeSensor, launcherSensor;

  /**
   * Creates a new IndexerSubsystem
   */
  public IndexerSubsystem() {
    indexerSpark = new SparkMax(IndexerConstants.kMotorPort, IndexerConstants.kMotorType);

    indexerSpark.configure(Configs.IndexerSubsystem.indexerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    controller = indexerSpark.getClosedLoopController();
    encoder = indexerSpark.getEncoder();

    intakeSensorTrigger = new DigitalOutput(IndexerConstants.kIntakeSensorTriggerPin);
    launcherSensorTrigger = new DigitalOutput(IndexerConstants.kLauncherSensorTriggerPin);

    intakeSensor = new UltrasonicSensor(IndexerConstants.kIntakeSensorChannel,
        () -> (5 / RobotController.getVoltage5V()) * 0.125);
    launcherSensor = new UltrasonicSensor(IndexerConstants.kLauncherSensorChannel,
        () -> (5 / RobotController.getVoltage5V()) * 0.125);

    turnOnIntakeSensor();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/Intake", getIntakeSensor());
    Logger.recordOutput("Indexer/Launcher", getLauncherSensor());
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

  /**
   * Runs the indexer towards a subsystem.
   * 
   * @param indexVelocity The velocity to run the indexer. Will determine the
   *                      direction to run at, therefore which subsystem to run
   *                      to.
   * 
   * @return A command requiring the indexer.
   */
  public Command indexToSubsystem(DoubleSupplier indexVelocity) {
    return Commands.runEnd(() -> {
      if (Math.signum(indexVelocity.getAsDouble()) == Math.signum(IndexerConstants.kDirectionConstant)) {
        // Move towards the launcher
        if (getLauncherSensor()) {
          setVelocity(0);
          return;
        }
      } else {
        // Move towards the intake
        if (getIntakeSensor()) {
          setVelocity(0);
          return;
        }
      }
      setVelocity(indexVelocity.getAsDouble());
    }, () -> setVelocity(0), this);
  }

  /**
   * Runs the indexer at the same velocity as another subsystem.
   * 
   * @param velocity The velocity to run at
   * @return A command requiring the indexer.
   */
  public Command followVelocity(DoubleSupplier velocity) {
    return Commands.runEnd(() -> setVelocity(velocity.getAsDouble()), () -> setVelocity(0), this);
  }

  /**
   * Manually control the speed of the indexer
   * 
   * @param input A joystick input between -1.0 and +1.0
   * @return A command requiring the indexer.
   */
  public Command manualSpeed(DoubleSupplier input) {
    return Commands.run(() -> setSpeed(input.getAsDouble()), this);
  }

  public void turnOnIntakeSensor() {
    intakeSensorTrigger.set(true);
    launcherSensorTrigger.set(false);
  }

  public void turnOnLauncherSensor() {
    intakeSensorTrigger.set(false);
    launcherSensorTrigger.set(true);
  }

  public void turnOffBothSensors() {
    intakeSensorTrigger.set(false);
    launcherSensorTrigger.set(false);
  }
}
