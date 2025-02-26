package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakSensor implements IndexerSensor {
    private final DigitalInput sensor;

    public BeamBreakSensor(int channel) {
        sensor = new DigitalInput(channel);
    }

    @Override
    public boolean triggered() {
        return sensor.get();
    }

}
