package frc.robot.subsystems.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor implements IndexerSensor {
  private final AnalogInput sensor;

  private DoubleSupplier scaleFactor;

  public UltrasonicSensor(int channel, DoubleSupplier scaleFactor) {
    sensor = new AnalogInput(channel);
    this.scaleFactor = scaleFactor;
  }

  /**
   * {@inheritDoc}
   */
  public boolean triggered() {
    return (sensor.getValue() * scaleFactor.getAsDouble()) < 32;// * voltageScaleFactor * 0.125;
  }
}
