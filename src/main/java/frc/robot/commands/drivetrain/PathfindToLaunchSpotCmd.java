// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;

public class PathfindToLaunchSpotCmd extends SequentialCommandGroup {
  /** Creates a new PathfindToLaunchSpotCmd. */
  public PathfindToLaunchSpotCmd() {
    addCommands(
        AutoBuilder.pathfindToPose(AutoConstants.kLaunchPose, AutoConstants.kPathfindingConstraints));
  }
}
