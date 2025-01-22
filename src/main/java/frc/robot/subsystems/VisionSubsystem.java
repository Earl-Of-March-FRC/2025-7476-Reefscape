// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable networkTable;
  private double[] pythonEntry = {};

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    // Send data to Python
    double[] dataToSend = { ((Constants.LimelightConstants.hasBorders) ? 1 : 0) };
    LimelightHelpers.setPythonScriptData("", dataToSend);
    networkTable = NetworkTableInstance.getDefault().getTable(LimelightConstants.kNetworkTableKey);

  }

  @Override
  public void periodic() {

    pythonEntry = networkTable.getEntry(LimelightConstants.kNetworkTableEntry).getDoubleArray(new double[] {});

    // SmartDashboard.putNumber("Limelight x", pythonEntry[0]);
    // SmartDashboard.putNumber("Limelight y", pythonEntry[1]);
    // SmartDashboard.putNumber("Limelight w", pythonEntry[2]);
    // SmartDashboard.putNumber("Limelight h", pythonEntry[3]);
    SmartDashboard.putNumber("Limelight distance", pythonEntry[0]);
    SmartDashboard.putNumber("Limelight H-angle", pythonEntry[0]);
    SmartDashboard.putNumber("Limelight V-angle", pythonEntry[0]);

    // This method will be called once per scheduler run
  }
}
