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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
  private final NetworkTable networkTable;
  private final Supplier<Pose2d> drivetrainPoseSupplier;
  private double x_distance, x_angle, y_angle;
  private Pose2d relativeToRobot = new Pose2d(), relativeToField = new Pose2d(); // In cm

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
    x_distance = networkTable.getEntry("x_distance").getDouble(0); // Distance of the ball from the camera (y on Pose2D)
    x_angle = networkTable.getEntry("x_angle").getDouble(0); // Ball left of camera = -, Ball right of camera = +
    y_angle = networkTable.getEntry("y_angle").getDouble(0); // Ball below camera = -, Ball above camera = +

    /*
     * Math for the x parameter of the Pose2d
     * O/A = tan(theta)
     * A = x_distance
     * O = ???
     * theta = x_angle
     * O/x_distance = tan(x_angle)
     * 
     * O = tan(x_angle) * x_distance
     * 
     * We also account for the camera offset by subtracting them.
     */
    relativeToRobot = new Pose2d(
        ((Math.tan(x_angle) * x_distance) - AlgaeConstants.camera1X) * 0.01,
        (x_distance - AlgaeConstants.camera1Z) * 0.01,
        Rotation2d.kZero); // Algae's rotation has no effect

    // Assuming we already have the robot's position on the field (provided by the
    // Pose2d supplier), we can calculate the game piece's position on the field by
    // simply adding the game piece's robot-relative coordinates to the robot's
    // field-relative coordinates (with the robot's angle in mind). This can easily
    // be done using WPILib's built in Translation2d and Transform2d classes.
    relativeToField = drivetrainPoseSupplier.get()
        .plus(new Transform2d(relativeToRobot.getTranslation(), relativeToRobot.getRotation()));

    Logger.recordOutput("Vision/Algae/RelativeRobot", relativeToRobot);
    Logger.recordOutput("Vision/Algae/RelativeField", relativeToField);
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
    Pose2d overshoot = new Pose2d(1.0, 1.0, relativeToField.getRotation());
    /*
     * Use TWO waypoints:
     * * Algae Position
     * * Algae Position + Overshoot
     */
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        // WARNING I suspect that this should be getRelativeToRobot() to get an
        // overshoot, get the difference vector, extend its magnitude by TODO ???
        relativeToField,
        // check for negativity??
        // TODO avoid magnitudes, do vector math more directly
        new Pose2d(relativeToField.getMeasureX().magnitude() + overshoot.getMeasureX().magnitude(),
            relativeToField.getMeasureY().magnitude() + overshoot.getMeasureY().magnitude(),
            relativeToField.getRotation()));

    // TODO add constraints
    PathConstraints constraints = new PathConstraints(2.0, 1.0, 2 * Math.PI, 4 * Math.PI);

    // TODO fix this line of code.
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
