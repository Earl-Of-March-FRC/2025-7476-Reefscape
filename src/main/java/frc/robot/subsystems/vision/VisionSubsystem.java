// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.limelight.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  private final NetworkTable networkTable;
  private double[] pythonEntry = {};

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    // Send data to Python
    double[] dataToSend = { ((Constants.LimelightConstants.hasBorders) ? 2 : 1) }; // 2 is true 1 is false
    LimelightHelpers.setPythonScriptData("", dataToSend);
    networkTable = NetworkTableInstance.getDefault().getTable(LimelightConstants.kNetworkTableKey);
  }

  @Override
  public void periodic() {

    pythonEntry = networkTable.getEntry(LimelightConstants.kNetworkTableEntry).getDoubleArray(new double[] {});
    double tx = networkTable.getEntry("tx").getDouble(0);
    double ty = networkTable.getEntry("ty").getDouble(0);
    /*
     * SmartDashboard.putNumber("Horizontal Offset", tx);
     * SmartDashboard.putNumber("Vertical Offset", ty);
     * 
     * SmartDashboard.putNumber("Limelight distance", pythonEntry[0]);
     * SmartDashboard.putNumber("Limelight H-angle", pythonEntry[1]);
     * SmartDashboard.putNumber("Limelight V-angle", pythonEntry[2]);
     * 
     * // SmartDashboard.putNumber("Horizontal Offset", aprilTagEntry[0]);
     * // SmartDashboard.putNumber("Vertical Offset", aprilTagEntry[1]);
     */
    // This method will be called once per scheduler run
  }

  public void setPipeline(int pipelineIndex) {
    LimelightHelpers.setPipelineIndex("", pipelineIndex);
  }

}
