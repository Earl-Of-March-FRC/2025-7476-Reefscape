// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable networkTable;
  private double[] pythonEntry = {};

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    networkTable = NetworkTableInstance.getDefault().getTable(LimelightConstants.kNetworkTableKey);
  }

  @Override
  public void periodic() {
    pythonEntry = networkTable.getEntry(LimelightConstants.kNetworkTableEntry).getDoubleArray(new double[] {});
    // This method will be called once per scheduler run
  }
}
