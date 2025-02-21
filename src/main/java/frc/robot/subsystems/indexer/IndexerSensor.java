package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

/**
 * The sensor interface for the IndexerSubsystem. This was created due to the
 * team's confusing switches between a Beam Break and an ultrasonic sensor.
 */
public interface IndexerSensor {
  default String getName() {
    return "Sensor";
  }

  /**
   * Check if the sensor was triggered by the algae.
   * 
   * @return {@code true} if the sensor was triggered or sensor not found.
   *         {@code false} if not.
   */
  default boolean triggered() {
    return true;
  }

  /**
   * Periodically log to AdvantageKit
   */
  default void periodic() {
    Logger.recordOutput("Indexer/" + getName(), triggered());
  };
}
