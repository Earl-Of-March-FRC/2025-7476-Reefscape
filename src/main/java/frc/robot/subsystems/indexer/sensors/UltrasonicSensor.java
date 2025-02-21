package frc.robot.subsystems.indexer.sensors;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltrasonicSensor implements IndexerSensor {
  private final String name;
  private final AnalogInput sensor;

  private DoubleSupplier scaleFactor;

  public UltrasonicSensor(String name, int channel, DoubleSupplier scaleFactor) {
    this.name = name;
    sensor = new AnalogInput(channel);
    this.scaleFactor = scaleFactor;
  }

  @Override
  public String getName() {
    return name;
  }

  public boolean triggered() {
    return (sensor.getValue() * scaleFactor.getAsDouble()) < 32;// * voltageScaleFactor * 0.125;
  }
}
