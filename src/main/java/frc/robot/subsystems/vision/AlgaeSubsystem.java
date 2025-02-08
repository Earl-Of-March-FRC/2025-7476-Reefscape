// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
  private final NetworkTable networkTable;
  private double x_distance, x_angle, y_angle;
  private Pose2d relativeToRobot = new Pose2d(); // In cm

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
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
     */
    relativeToRobot = new Pose2d(Math.tan(x_angle) * x_distance, x_distance, new Rotation2d());
    System.out.println(relativeToRobot.getX() + ", " + relativeToRobot.getY());
    Logger.recordOutput("Vision/AlgaeRelativeRobot", relativeToRobot);
  }

  /**
   * Get the position of the algae (in centimeters) relative to the robot
   * 
   * @return {@code Pose2d} storing the x and y coordinates of a detected algae in
   *         centimeters
   */
  public Pose2d getRelativeToRobot() {
    return relativeToRobot;
  }

}
