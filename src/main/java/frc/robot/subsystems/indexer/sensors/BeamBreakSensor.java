package frc.robot.subsystems.indexer.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakSensor implements IndexerSensor {
  private final String name;
  private final DigitalInput sensor;

  public BeamBreakSensor(String name, int channel) {
    this.name = name;
    sensor = new DigitalInput(channel);
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public boolean triggered() {
    return !sensor.get();
  }
}
