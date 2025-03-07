// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AlgaeSubsystem extends SubsystemBase {
  private Drivetrain drivetrain;
  private final Supplier<Pose2d> drivetrainPoseSupplier;
  private Pose2d relativeToRobot = new Pose2d(), relativeToField = new Pose2d(); // In m
  private PhotonCamera camera1;
  private Pose2d robotPose2d;

  /**
   * Creates a new AlgaeSubsystem in which creates a new relative to robot object
   * 
   * @param drivetrainPoseSupplier Supplies the drivetrain's field-relative
   *                               position
   */
  public AlgaeSubsystem(Supplier<Pose2d> drivetrainPoseSupplier) {
    camera1 = new PhotonCamera(Constants.Vision.PhotonConstants.kCamera1);
    this.drivetrainPoseSupplier = drivetrainPoseSupplier;
    relativeToRobot = new Pose2d();
  }

  @Override
  public void periodic() {
    // updateTargetPose();
    Logger.recordOutput("Vision/Algae/RelativeRobot", relativeToRobot);
    Logger.recordOutput("Vision/Algae/RelativeField", relativeToField);
  }

  public void updateTargetPose() {
    var result = camera1.getLatestResult();
    boolean hasTarget = result.hasTargets();

    if (hasTarget) {
      PhotonTrackedTarget target = result.getBestTarget();

      double targetYaw = target.getYaw(); // in degrees (x angle)
      Transform3d cameraToTarget = target.bestCameraToTarget;

      Transform3d robotToAlgae = cameraToTarget.plus(Constants.Vision.PhotonConstants.robotToCamera);

      SmartDashboard.putBoolean("Algae Detected",
          targetYaw < Constants.Vision.AlgaeConstants.kUpperBound
              && targetYaw > Constants.Vision.AlgaeConstants.kLowerBound);

      relativeToRobot = new Pose2d(
          robotToAlgae.getTranslation().getX(),
          robotToAlgae.getTranslation().getY(),
          Rotation2d.kZero);

      // Assuming we already have the robot's position on the field
      relativeToField = drivetrainPoseSupplier.get()
          .plus(new Transform2d(relativeToRobot.getTranslation(), relativeToRobot.getRotation()));
    } else {
      SmartDashboard.putBoolean("Algae Detected", false);
    }
  }

  /**
   * Get the position of the algae (in meters) relative to the robot
   * 
   * @return {@code Pose2d} storing the x and y coordinates of a detected algae in
   *         meters
   * @see #getRelativeToField
   */
  public Pose2d getRelativeToRobot() {
    return relativeToRobot;
  }

  /**
   * Get the position of the algae (in meters) relative to the field
   * 
   * @return {@code Pose2d} storing the x and y coordinates of a detected algae in
   *         meters
   * @see #getRelativeToRobot
   */
  public Pose2d getRelativeToField() {
    return relativeToField;
  }

  /**
   * This function returns a path so that the robot can intake the algae!
   * Path will use two waypoints:
   * * Algae Position
   * * Algae Position + Overshoot
   * 
   * @return PathPlannerPath the path the robot should follow to eat the algae
   * @experimental
   */
  public PathPlannerPath getPath() {
    updateTargetPose();

    Translation2d currentTranslation2d = drivetrain.getPose().getTranslation();

    double overshootDistance = 1.0; // Distance to overshoot the algae (adjust when testing) -- 1.0 units of
                                    // overshot distance

    // this will be the direction/path the robot will travel in to get to the algae
    Translation2d distanceToAlgae = relativeToField.getTranslation().minus(currentTranslation2d);

    // what angle should the robot face so that it can intake the algae?
    // NOTE: the intake is on the front (0 deg) of the robot
    Rotation2d desiredAngle = distanceToAlgae.getAngle();

    /*
     * Calculate the overshoot position by extending the algae position in the
     * direction of the robot's current rotation algaePose.getTranslation() gives us
     * the current position of the algae.
     * Math.cos and Math.sin are used to calculate the direction in which the robot
     * is facing (based on its rotation), then we multiply that direction by the
     * overshoot distance to get how far to extend the position along that
     * direction.
     */
    Translation2d overshootTranslation = this.getRelativeToField().getTranslation().plus(new Translation2d(
        Math.cos(this.getRelativeToField().getRotation().getRadians()) * overshootDistance, // Extend in the X direction
        Math.sin(this.getRelativeToField().getRotation().getRadians()) * overshootDistance // Extend in the Y direction
    ));

    /*
     * Create the overshoot Pose2d, which represents the position (with translation)
     * and rotation.
     * Pose2d combines the translation (position) and rotation (angle) into a single
     * object.
     */
    Pose2d overshootPose = new Pose2d(overshootTranslation, desiredAngle);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        // Starting point: The robot's current position on the field

        // Algae point: The algae's position, +-20cm
        // NOTE: the robot needs to rotate so that its intake faces the algae
        new Pose2d(relativeToField.getTranslation(), desiredAngle),
        // Overshoot point: The position after overshooting the algae
        overshootPose);

    PathConstraints constraints = new PathConstraints(2.0, 1.0, 2 * Math.PI, 4 * Math.PI);

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        null);
    // new GoalEndState(0.0, Rotation2d.fromDegrees(-90))

    // waypoints.get(1) as a backup
    path.getEventMarkers().add(new EventMarker("Intake", 1.0, 2.0, null));

    // Prevent the path from being flipped if the coordinates are already correct
    // (this was copy pasted from
    // https://pathplanner.dev/pplib-create-a-pat(h-on-the-fly.html)
    path.preventFlipping = true;

    return path;
  }

  public void setPipeline(int index) {
    camera1.setPipelineIndex(index);
  }

}
