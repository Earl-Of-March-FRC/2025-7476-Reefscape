// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase implements IndexerInterface {
  public IndexerSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void setSpeed(double percent) {
    throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
  }

  @Override
  public void setVoltage(double voltage) {
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }
}
