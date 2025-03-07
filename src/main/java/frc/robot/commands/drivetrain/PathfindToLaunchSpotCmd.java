// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;

public class PathfindToLaunchSpotCmd extends SequentialCommandGroup {
  /** Creates a new PathfindToLaunchSpotCmd. */
  public PathfindToLaunchSpotCmd() {

    // Determine launch pose to use based on alliance
    Pose2d launchPose;

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      launchPose = AutoConstants.kLaunchPoseBlue;
    } else {
      launchPose = AutoConstants.kLaunchPoseRed;
    }

    addCommands(
        AutoBuilder.pathfindToPose(launchPose, AutoConstants.kPathfindingConstraints));
  }
}
