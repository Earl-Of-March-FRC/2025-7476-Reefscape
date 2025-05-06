package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import com.pathplanner.lib.path.Waypoint;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.vision.AlgaeSubsystem;
import frc.robot.subsystems.vision.AlgaeSubsystem.InterceptionResult;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class InterceptAlgaeCmd extends Command {
  private final Drivetrain drive;
  private final AlgaeSubsystem algaeSubsystem;
  private final PathConstraints constraints;

  private Timer updateTimer = new Timer();
  private Command currentPathCommand = null;
  private Translation2d lastInterceptionPoint = null;
  private boolean isExecutingPath = false;

  /**
   * Creates a command that uses PathPlanner to intercept the algae
   * 
   * @param drive           The drive subsystem
   * @param algaeSubsystem  The algae vision subsystem
   * @param maxVelocity     Maximum velocity for path planning (m/s)
   * @param maxAcceleration Maximum acceleration for path planning (m/s²)
   */
  public InterceptAlgaeCmd(
      Drivetrain drive,
      AlgaeSubsystem algaeSubsystem,
      double maxVelocity,
      double maxAcceleration) {
    this.drive = drive;
    this.algaeSubsystem = algaeSubsystem;
    this.constraints = new PathConstraints(
        maxVelocity,
        maxAcceleration,
        Math.PI, // Max angular velocity (rad/s)
        Math.PI * 2 // Max angular acceleration (rad/s²)
    );

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    updateTimer.reset();
    updateTimer.start();
    lastInterceptionPoint = null;
    isExecutingPath = false;

    // Calculate initial interception and start path
    updateInterceptionPath();
  }

  @Override
  public void execute() {
    // Check if it's time to update the interception path
    if (updateTimer.hasElapsed(Constants.Vision.AlgaeInterceptionConstants.interceptionUpdateInterval)) {
      updateTimer.reset();

      // Only update if we have algae data and need a new path
      if (algaeSubsystem.getRelativeToField() != null) {
        updateInterceptionPath();
      }
    }
  }

  /**
   * Calculates a new interception point and updates the robot's path if needed
   */
  private void updateInterceptionPath() {
    // Get current robot state
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds chassisSpeeds = drive.getChassisSpeedsFieldRelative();
    Translation2d robotVelocity = new Translation2d(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond);

    // Get current algae state
    Pose2d algaePose = algaeSubsystem.getRelativeToField();
    Translation2d algaeVelocity = algaeSubsystem.getAlgaeVelocity();
    double algaeDecel = algaeSubsystem.getAlgaeDeceleration();

    if (algaePose == null) {
      return; // No algae detected
    }

    // Calculate interception
    InterceptionResult interception = algaeSubsystem.calculateInterception(
        robotPose,
        robotVelocity,
        algaePose,
        algaeVelocity,
        constraints.maxAccelerationMPSSq(),
        constraints.maxVelocityMPS(),
        algaeDecel,
        Constants.Vision.AlgaeInterceptionConstants.maxTime,
        Constants.Vision.AlgaeInterceptionConstants.dt);

    // Check if we have a valid interception point
    if (interception.point == null) {
      return; // No valid interception possible
    }

    // Check if the new point is significantly different from the last one
    if (lastInterceptionPoint != null) {
      double distance = interception.point.getDistance(lastInterceptionPoint);
      if (distance < Constants.Vision.AlgaeInterceptionConstants.minUpdatedPointDistance && isExecutingPath) {
        return; // Skip update - point hasn't moved enough
      }
    }

    // Store the new interception point
    lastInterceptionPoint = interception.point;

    // Cancel current path command if one is running
    if (currentPathCommand != null && !currentPathCommand.isFinished()) {
      currentPathCommand.cancel();
    }

    // Generate heading based on algae velocity direction
    Rotation2d targetHeading;
    if (algaeVelocity.getNorm() > 0.1) {
      // Point toward algae's direction of travel
      targetHeading = new Rotation2d(algaeVelocity.getX(), algaeVelocity.getY());
    } else {
      // Point toward the algae if it's nearly stationary
      Translation2d toAlgae = algaePose.getTranslation().minus(robotPose.getTranslation());
      targetHeading = new Rotation2d(toAlgae.getX(), toAlgae.getY());
    }

    // Create path
    PathConstraints pathConstraints = new PathConstraints(
        constraints.maxVelocityMPS(),
        constraints.maxAccelerationMPSSq(),
        constraints.maxAngularVelocityRadPerSec(),
        constraints.maxAngularAccelerationRadPerSecSq());

    // Create waypoints for the path
    List<Waypoint> waypoints = new ArrayList<>();

    // Starting waypoint - need prevControl, anchor, nextControl as Translation2d
    // objects
    Translation2d startPos = robotPose.getTranslation();
    Translation2d endPos = interception.point;

    // Calculate control points for smooth path
    // Direction vector from start to end
    Translation2d dirVector = endPos.minus(startPos);
    double distance = dirVector.getNorm();

    // Control points at approximately 1/3 of the distance
    Translation2d startNextControl = startPos.plus(dirVector.times(0.33));
    Translation2d endPrevControl = startPos.plus(dirVector.times(0.67));

    // For the first waypoint, prevControl could be behind the robot
    Translation2d startPrevControl = startPos.minus(new Translation2d(
        Math.cos(robotPose.getRotation().getRadians()) * distance * 0.33,
        Math.sin(robotPose.getRotation().getRadians()) * distance * 0.33));

    // For the last waypoint, nextControl continues in target heading direction
    Translation2d endNextControl = endPos.plus(new Translation2d(
        Math.cos(targetHeading.getRadians()) * distance * 0.33,
        Math.sin(targetHeading.getRadians()) * distance * 0.33));

    // Add waypoints with control points
    waypoints.add(new Waypoint(startPrevControl, startPos, startNextControl));
    waypoints.add(new Waypoint(endPrevControl, endPos, endNextControl));

    // Create path with the correct constructor
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        pathConstraints,
        null, // ideal starting state - null for default
        new GoalEndState(0.1, targetHeading), // End with slow speed and target heading
        false // not reversed
    );

    // Create and schedule the path following command
    currentPathCommand = AutoBuilder.followPath(path);
    currentPathCommand.schedule();
    isExecutingPath = true;
  }

  /**
   * Creates a set of Bezier control points for a smooth path from start to end
   */

  @Override
  public void end(boolean interrupted) {
    // Stop following path if the command is interrupted
    if (currentPathCommand != null && !currentPathCommand.isFinished()) {
      currentPathCommand.cancel();
    }

    // Safety - stop the drive
    ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0, 0, 0);
    drive.runVelocityFieldRelative(zeroSpeeds);
    updateTimer.stop();
    isExecutingPath = false;
  }

  @Override
  public boolean isFinished() {
    // If we have a valid interception point and we're close enough to it
    if (lastInterceptionPoint != null) {
      double distanceToTarget = drive.getPose().getTranslation().getDistance(lastInterceptionPoint);
      return distanceToTarget < Constants.Vision.AlgaeInterceptionConstants.COLLISION_THRESHOLD;
    }
    return false;
  }
}