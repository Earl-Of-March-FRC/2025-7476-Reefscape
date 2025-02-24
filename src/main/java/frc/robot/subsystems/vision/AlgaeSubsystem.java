// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
  private final NetworkTable networkTable;
  private final Supplier<Pose2d> drivetrainPoseSupplier;
  private Pose2d relativeToRobot = new Pose2d(), relativeToField = new Pose2d(); // In m

  /**
   * Creates a new AlgaeSubsystem
   * 
   * @param drivetrainPoseSupplier Supplies the drivetrain's field-relative
   *                               position
   */
  public AlgaeSubsystem(Supplier<Pose2d> drivetrainPoseSupplier) {
    this.drivetrainPoseSupplier = drivetrainPoseSupplier;
    networkTable = NetworkTableInstance.getDefault().getTable(AlgaeConstants.kNetworkTableKey);
    relativeToRobot = new Pose2d();
  }

  @Override
  public void periodic() {
    updateTargetPose();
    Logger.recordOutput("Vision/Algae/RelativeRobot", relativeToRobot);
    Logger.recordOutput("Vision/Algae/RelativeField", relativeToField);
  }

  public void updateTargetPose() {

    boolean hasTarget = networkTable.getEntry("hasTarget").getBoolean(false);

    if (hasTarget) {
      double targetYaw = networkTable.getEntry("targetYaw").getDouble(0.0); // in degrees (x angle)
      double[] targetPose = networkTable.getEntry("targetPose").getDoubleArray(new double[7]);

      double xDistance = targetPose[0];

      relativeToRobot = new Pose2d(
          ((Math.tan(targetYaw) * xDistance) - AlgaeConstants.camera1X) * 0.01,
          (xDistance - AlgaeConstants.camera1Z) * 0.01,
          Rotation2d.kZero); // Algae's rotation has no effect

      // Assuming we already have the robot's position on the field
      relativeToField = drivetrainPoseSupplier.get()
          .plus(new Transform2d(relativeToRobot.getTranslation(), relativeToRobot.getRotation()));
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
   * Calling this function will call pathplanner to drive to the algae AND intake!
   * Path will use two waypoints:
   * * Algae Position
   * * Algae Position + Overshoot
   * TODO move constants to constant file
   * TODO consider not using local variable "relativeToField"
   * TODO call commands to intake
   * 
   * @experimental
   */
  public void intakeAlgae() {
    double overshootDistance = 1.0; // Distance to overshoot the algae (adjust when testing) -- 1.0 units of
                                    // overshot distance

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
    Pose2d overshootPose = new Pose2d(overshootTranslation, this.getRelativeToField().getRotation());

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        // Starting point: The robot's current position on the field

        // Algae point: The algae's position, +-20cm
        // NOTE: the robot needs to rotate so that its intake faces the algae
        // TODO make sure the swerve rotation is OK
        relativeToField,
        // Overshoot point: The position after overshooting the algae
        overshootPose);

    // TODO tune constraints
    PathConstraints constraints = new PathConstraints(2.0, 1.0, 2 * Math.PI, 4 * Math.PI);

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        null);
    // new GoalEndState(0.0, Rotation2d.fromDegrees(-90))

    // TODO verify if 1.0 & 2.0 work as expected. Try waypoints.get(0),
    // waypoints.get(1) as a backup
    // TODO add intake commands (replace fourth parameter of "null")
    // TODO rename named command name ("Intake") to something meaningful
    path.getEventMarkers().add(new EventMarker("Intake", 1.0, 2.0, null));

    // Prevent the path from being flipped if the coordinates are already correct
    // (this was copy pasted from
    // https://pathplanner.dev/pplib-create-a-path-on-the-fly.html)
    // TODO verify if this is helpful
    path.preventFlipping = true;
  }

}
