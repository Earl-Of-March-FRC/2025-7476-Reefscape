// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;

public class PathfindToLaunchSpotCmd extends SequentialCommandGroup {
  /** Creates a new PathfindToLaunchSpotCmd. */
  public PathfindToLaunchSpotCmd(Pose2d launchPose) {
    addCommands(
        AutoBuilder.pathfindToPose(launchPose, DriveConstants.kPathfindingConstraints));
  }
}
