// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
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
     * Use TWO waypoints:
     * * Algae Position
     * * Algae Position + Overshoot
     */

    /*
     * Eric, this code does not work because it adds the magnitudes of the
     * relativeToField
     * and overshoot translation components directly, which results in a positional
     * shift rather than extending the overshoot in the correct direction. To
     * achieve the desired overshoot, you should calculate the direction vector from
     * the robot's current position and then apply the overshoot distance in that
     * direction, rather than simply adding magnitudes.
     * 
     * 
     * List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
     * // WARNING I suspect that this should be getRelativeToRobot() to get an
     * // overshoot, get the difference vector, extend its magnitude by TODO ???
     * relativeToField,
     * // check for negativity??
     * // TODO avoid magnitudes, do vector math more directly
     * new Pose2d(relativeToField.getMeasureX().magnitude() +
     * overshoot.getMeasureX().magnitude(),
     * relativeToField.getMeasureY().magnitude() +
     * overshoot.getMeasureY().magnitude(),
     * relativeToField.getRotation()));
     */

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
        relativeToField, // Starting point: The robot's current position on the field
        overshootPose // Overshoot point: The position after overshooting the algae
    );

    PathConstraints constraints = new PathConstraints(2.0, 1.0, 2 * Math.PI, 4 * Math.PI); // Will need to tune the
                                                                                           // constraints

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can
              // be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If
                                                           // using a differential drivetrain, the rotation will have no
                                                           // effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    // (this was copy pasted from
    // https://pathplanner.dev/pplib-create-a-path-on-the-fly.html)
    // TODO verify if this is helpful
    path.preventFlipping = true;
  }

}
