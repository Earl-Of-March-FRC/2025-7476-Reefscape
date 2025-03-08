// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DriveConstants.LaunchingDistances;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToNearestBargeLaunchingZoneCmd extends InstantCommand {
  private final Drivetrain driveSub;

  /** Creates a new MoveToNearestBargeLaunchingZoneCmd. */
  public MoveToNearestBargeLaunchingZoneCmd(
      Drivetrain driveSub) {
    this.driveSub = driveSub;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d startingPose = driveSub.getPose();
    boolean onBlueSide = driveSub.isOnBlueSide();
    double targetRadians;
    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      targetRadians = (onBlueSide == (alliance == Alliance.Blue)) ? 0 : Math.PI;
    } else {
      targetRadians = startingPose.getRotation().getRadians();
    }
    Pose2d targetPose = new Pose2d(
        FieldConstants.kBargeX + (onBlueSide ? -1 : 1) * LaunchingDistances.kMetersFromBarge, startingPose.getX(),
        new Rotation2d(targetRadians));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startingPose, targetPose);
    PathPlannerPath path = new PathPlannerPath(waypoints, DriveConstants.kPathfindingConstraints, null,
        new GoalEndState(0, Rotation2d.fromRadians(targetRadians)));
    path.preventFlipping = true;

    AutoBuilder.followPath(path).schedule();
  }
}
