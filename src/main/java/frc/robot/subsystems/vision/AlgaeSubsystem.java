// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import frc.robot.Constants.Vision.PhotonConstants;

public class AlgaeSubsystem extends SubsystemBase {
  // private Drivetrain drivetrain;
  private final Supplier<Pose2d> drivetrainPoseSupplier;
  private Pose2d relativeToRobot, relativeToField; // In m
  private Translation2d algaeVelocity = new Translation2d(0, 0); // In m/s
  private Translation2d lastAlgaeVelocity = new Translation2d(0, 0); // Previous velocity for accel calc
  private double algaeDecel = 0.0; // Estimated deceleration in m/s²
  private double lastUpdateTime = 0;
  private Pose2d lastAlgaePosition = null;
  private double velocityHistoryTimeWindow = 0.5; // Time window in seconds for velocity averaging
  private final int VELOCITY_HISTORY_SIZE = 5; // Number of velocity samples to keep
  private Translation2d[] velocityHistory = new Translation2d[VELOCITY_HISTORY_SIZE];
  private double[] velocityTimestamps = new double[VELOCITY_HISTORY_SIZE];
  private int velocityHistoryIndex = 0;

  private PhotonCamera camera1;

  final double COLLISION_THRESHOLD = 0.3; // 30cm threshold for interception

  /**
   * Creates a new AlgaeSubsystem in which creates a new relative to robot object
   * 
   * @param drivetrainPoseSupplier Supplies the drivetrain's field-relative
   *                               position
   */
  public AlgaeSubsystem(Supplier<Pose2d> drivetrainPoseSupplier) {
    this.drivetrainPoseSupplier = drivetrainPoseSupplier;
    camera1 = new PhotonCamera(Constants.Vision.PhotonConstants.kCamera1);
  }

  @Override
  public void periodic() {
    updateTargetPose();
    Logger.recordOutput("Vision/Algae/RelativeRobot", relativeToRobot);
    Logger.recordOutput("Vision/Algae/RelativeField", relativeToField);
    Logger.recordOutput("Vision/Algae/Velocity", new double[] { algaeVelocity.getX(), algaeVelocity.getY() });
    Logger.recordOutput("Vision/Algae/Deceleration", algaeDecel);
  }

  private boolean updateTargetPose() {
    var results = camera1.getAllUnreadResults();
    // If no targets, return false
    if (results.isEmpty() || !results.get(0).hasTargets()) {
      SmartDashboard.putBoolean("Algae Detected", false);
      return false;
    }

    var result = results.get(0);
    double currentTime = result.getTimestampSeconds();

    PhotonTrackedTarget target = result.getBestTarget();
    double targetYaw = target.getYaw();
    Transform3d robotToAlgae = target.bestCameraToTarget.plus(PhotonConstants.kRobotToCam1);

    SmartDashboard.putBoolean("Algae Detected",
        targetYaw < Constants.Vision.AlgaeConstants.kUpperBound
            && targetYaw > Constants.Vision.AlgaeConstants.kLowerBound);

    relativeToRobot = new Pose2d(robotToAlgae.getTranslation().getX(), robotToAlgae.getTranslation().getY(),
        Rotation2d.kZero);

    relativeToField = drivetrainPoseSupplier.get()
        .plus(new Transform2d(relativeToRobot.getTranslation(), relativeToRobot.getRotation()));

    // Update velocity estimation
    if (lastAlgaePosition != null && lastUpdateTime > 0) {
      double dt = currentTime - lastUpdateTime;
      if (dt > 0.001) { // Ensure we don't divide by near-zero
        Translation2d displacement = relativeToField.getTranslation().minus(lastAlgaePosition.getTranslation());

        // Store previous velocity for acceleration calculation
        lastAlgaeVelocity = algaeVelocity;

        // Calculate instantaneous velocity
        algaeVelocity = new Translation2d(displacement.getX() / dt, displacement.getY() / dt);

        // Store velocity in history buffer for smoothing and deceleration calculation
        velocityHistory[velocityHistoryIndex] = algaeVelocity;
        velocityTimestamps[velocityHistoryIndex] = currentTime;
        velocityHistoryIndex = (velocityHistoryIndex + 1) % VELOCITY_HISTORY_SIZE;

        // Calculate deceleration using velocity change
        // Only calculate deceleration if we're moving significantly
        double currentSpeed = Math.hypot(algaeVelocity.getX(), algaeVelocity.getY());
        double previousSpeed = Math.hypot(lastAlgaeVelocity.getX(), lastAlgaeVelocity.getY());

        if (currentSpeed > 0.1 && previousSpeed > 0.1) {
          // Calculate instantaneous deceleration from velocity difference
          // Only consider it deceleration if speed is decreasing
          if (currentSpeed < previousSpeed) {
            // Speed is decreasing, calculate deceleration magnitude
            double speedDelta = previousSpeed - currentSpeed;
            double instantDecel = speedDelta / dt;

            // Apply low-pass filter to smooth deceleration estimate
            // Use 80% of previous value and 20% of new measurement
            algaeDecel = 0.8 * algaeDecel + 0.2 * instantDecel;
          } else if (algaeDecel > 0 && currentSpeed > previousSpeed) {
            // Speed is increasing, gradually reduce deceleration estimate
            algaeDecel *= 0.9; // Decay factor
            if (algaeDecel < 0.05)
              algaeDecel = 0; // Reset to zero if very small
          }

          // Calculate more robust deceleration using linear regression on velocity
          // history
          calculateRobustDeceleration(currentTime);
        }

        // Log velocity and deceleration
        SmartDashboard.putNumber("Algae Velocity X", algaeVelocity.getX());
        SmartDashboard.putNumber("Algae Velocity Y", algaeVelocity.getY());
        SmartDashboard.putNumber("Algae Speed", currentSpeed);
        SmartDashboard.putNumber("Algae Deceleration", algaeDecel);
      }
    }

    // Store position for next velocity calculation
    lastAlgaePosition = relativeToField;
    lastUpdateTime = currentTime;

    return true;
  }

  /**
   * Calculates a more robust deceleration estimate using linear regression
   * on the velocity history. This helps filter out noise and provide a more
   * stable deceleration value.
   * 
   * @param currentTime The current timestamp for reference
   */
  private void calculateRobustDeceleration(double currentTime) {
    // Count valid entries in history
    int validEntries = 0;
    double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    double oldestTime = currentTime - velocityHistoryTimeWindow;

    // Calculate speeds for valid entries
    double[] speeds = new double[VELOCITY_HISTORY_SIZE];
    double[] times = new double[VELOCITY_HISTORY_SIZE];

    for (int i = 0; i < VELOCITY_HISTORY_SIZE; i++) {
      if (velocityHistory[i] != null && velocityTimestamps[i] > oldestTime) {
        speeds[validEntries] = Math.hypot(velocityHistory[i].getX(), velocityHistory[i].getY());
        // Normalize time to make calculation more numerically stable
        times[validEntries] = velocityTimestamps[i] - oldestTime;
        validEntries++;
      }
    }

    // Need at least 3 points for meaningful regression
    if (validEntries >= 3) {
      // Calculate linear regression parameters
      for (int i = 0; i < validEntries; i++) {
        sumX += times[i];
        sumY += speeds[i];
        sumXY += times[i] * speeds[i];
        sumXX += times[i] * times[i];
      }

      double n = validEntries;
      double slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);

      // Negative slope means deceleration is happening
      if (slope < 0) {
        // Convert slope to deceleration (absolute value since decel is always positive)
        double robustDecel = -slope; // slope is change in speed per time

        // Apply low-pass filter to smooth between instantaneous and robust calculation
        algaeDecel = 0.7 * algaeDecel + 0.3 * robustDecel;
      }
    }
  }

  public void setPipeline(int index) {
    camera1.setPipelineIndex(index);
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
   * Get the estimated velocity of the algae in m/s
   * 
   * @return Translation2d representing the velocity vector of the algae
   */
  public Translation2d getAlgaeVelocity() {
    return algaeVelocity;
  }

  /**
   * Get the estimated deceleration of the algae in m/s²
   * 
   * @return Double representing deceleration magnitude (always positive)
   */
  public double getAlgaeDeceleration() {
    return algaeDecel;
  }

  /**
   * Calculates the interception point between the robot and a moving algae
   * target.
   * This method implements a comprehensive approach that considers:
   * 1. Whether the algae will come to rest before interception
   * 2. Whether the robot can reach the algae considering velocity constraints
   * 3. Multiple motion scenarios (direct interception or interception after algae
   * stops)
   * 
   * @param robotPose     The current pose of the robot
   * @param robotVel      The current velocity of the robot (in m/s)
   * @param algaePose     The current pose of the algae
   * @param algaeVel      The current velocity of the algae (in m/s)
   * @param robotMaxAccel The maximum acceleration of the robot (in m/s²)
   * @param robotMaxVel   The maximum velocity of the robot (in m/s)
   * @param algaeDecel    The deceleration of the algae (in m/s²), 0 if constant
   *                      velocity
   * @param maxTime       Maximum time to simulate for interception (in seconds)
   * @param dt            Time step for numerical simulation (in seconds)
   * @return InterceptionResult containing the calculated interception point and
   *         time, or null values if not possible
   */
  public InterceptionResult calculateInterception(
      Pose2d robotPose,
      Translation2d robotVel,
      Pose2d algaePose,
      Translation2d algaeVel,
      double robotMaxAccel,
      double robotMaxVel,
      double algaeDecel,
      double maxTime,
      double dt) {

    // Extract positions from poses
    Translation2d robotPos = robotPose.getTranslation();
    Translation2d algaePos = algaePose.getTranslation();

    // First, determine if the algae will stop due to deceleration
    double algaeSpeed = Math.hypot(algaeVel.getX(), algaeVel.getY());

    // If algae has non-zero speed and positive deceleration
    Translation2d algaeFinalPos = null;
    double tStopAlgae = Double.POSITIVE_INFINITY;

    if (algaeSpeed > 0 && algaeDecel > 0) {
      // Time until algae stops
      tStopAlgae = algaeSpeed / algaeDecel;

      // Calculate algae position when it stops
      // For decelerated motion: s = v₀t - ½at²
      if (algaeSpeed > 0) {
        // Direction of algae motion
        Translation2d algaeDir = new Translation2d(
            algaeVel.getX() / algaeSpeed,
            algaeVel.getY() / algaeSpeed);

        // Distance traveled before stopping
        double stopDistance = algaeSpeed * tStopAlgae - 0.5 * algaeDecel * tStopAlgae * tStopAlgae;

        // Final algae position when stopped
        algaeFinalPos = new Translation2d(
            algaePos.getX() + algaeDir.getX() * stopDistance,
            algaePos.getY() + algaeDir.getY() * stopDistance);
      } else {
        // Algae already stopped
        algaeFinalPos = algaePos;
      }
    }

    // Try solving for interception after algae stops first (simpler case)
    if (tStopAlgae < Double.POSITIVE_INFINITY) {
      // Calculate time to reach the stationary algae position
      InterceptionResult interceptAfterStop = calculateTimeToPosition(
          robotPos, robotVel, algaeFinalPos, robotMaxAccel, robotMaxVel);

      if (interceptAfterStop != null) {
        double interceptTime = interceptAfterStop.time;
        Translation2d interceptPoint = interceptAfterStop.point;

        // Check if interception happens after algae has stopped
        if (interceptTime > tStopAlgae) {
          return new InterceptionResult(interceptPoint, interceptTime);
        }
      }
    }

    // For interception before algae stops or with continuous motion,
    // First, check if interception is even possible:
    // If algae is faster than robot's max speed and moving away
    if (algaeSpeed > robotMaxVel) {
      // Check if algae is moving away from robot
      double dx = algaePos.getX() - robotPos.getX();
      double dy = algaePos.getY() - robotPos.getY();
      double dotProduct = dx * algaeVel.getX() + dy * algaeVel.getY();

      if (dotProduct > 0) { // Moving away
        // Only possible if algae decelerates
        if (algaeDecel <= 0) {
          return new InterceptionResult(null, 0);
        }
      }
    }

    // Use numerical solution with differential equations for the general case
    return findInterceptNumerical(
        robotPos, robotVel, algaePos, algaeVel,
        robotMaxAccel, robotMaxVel, algaeDecel,
        maxTime, dt);
  }

  /**
   * Calculate the time required for the robot to reach a specific position
   * considering acceleration constraints and max velocity.
   * 
   * @param startPos     Starting position of the robot
   * @param startVel     Starting velocity of the robot
   * @param targetPos    Target position to reach
   * @param acceleration Robot acceleration magnitude
   * @param maxVel       Robot maximum velocity constraint
   * @return InterceptionResult containing calculated time and point, or null if
   *         not possible
   */
  private InterceptionResult calculateTimeToPosition(
      Translation2d startPos,
      Translation2d startVel,
      Translation2d targetPos,
      double acceleration,
      double maxVel) {

    // Vector from start to target
    double dx = targetPos.getX() - startPos.getX();
    double dy = targetPos.getY() - startPos.getY();
    double distance = Math.hypot(dx, dy);

    // If we're already at the target
    if (distance < 1e-6) {
      return new InterceptionResult(startPos, 0.0);
    }

    // Direction to target
    double dirX = dx / distance;
    double dirY = dy / distance;

    // Current velocity in the direction of the target
    double velInDirection = startVel.getX() * dirX + startVel.getY() * dirY;

    // Calculate time to come to a stop if moving away
    double tStop = 0;
    double stopDistance = 0;
    Translation2d newStartPos = startPos;

    if (velInDirection < 0) { // Moving away from target
      // Time to stop
      tStop = Math.abs(velInDirection) / acceleration;

      // Distance covered while stopping
      stopDistance = Math.abs(velInDirection * tStop / 2);

      // New starting position after stopping
      newStartPos = new Translation2d(
          startPos.getX() + dirX * (-stopDistance),
          startPos.getY() + dirY * (-stopDistance));

      // Recalculate displacement and distance
      dx = targetPos.getX() - newStartPos.getX();
      dy = targetPos.getY() - newStartPos.getY();
      distance = Math.hypot(dx, dy);

      // Update direction if needed
      if (distance > 1e-6) {
        dirX = dx / distance;
        dirY = dy / distance;
      }
    }

    // Time to reach max velocity
    double tMax = maxVel / acceleration;

    // Distance covered during acceleration phase
    double accelDistance = 0.5 * acceleration * tMax * tMax;

    // Calculate total time
    if (accelDistance >= distance) {
      // Target reached during acceleration phase
      double tAccel = Math.sqrt(2 * distance / acceleration);
      double totalTime = tStop + tAccel;
      return new InterceptionResult(targetPos, totalTime);
    } else {
      // Need to travel at max velocity for some time
      double remainingDistance = distance - accelDistance;
      double tConst = remainingDistance / maxVel;
      double totalTime = tStop + tMax + tConst;
      return new InterceptionResult(targetPos, totalTime);
    }
  }

  /**
   * Find interception using numerical integration of the motion equations.
   * Uses the same time step as the main simulation for consistency.
   *
   * @param robotPos      Initial robot position
   * @param robotVel      Initial robot velocity
   * @param algaePos      Initial algae position
   * @param algaeVel      Initial algae velocity
   * @param robotMaxAccel Maximum robot acceleration
   * @param robotMaxVel   Maximum robot velocity
   * @param algaeDecel    Algae deceleration rate
   * @param maxTime       Maximum simulation time
   * @param dt            Time step for simulation
   * @return InterceptionResult with calculated interception point and time, or
   *         null values if not possible
   */
  private InterceptionResult findInterceptNumerical(
      Translation2d robotPos,
      Translation2d robotVel,
      Translation2d algaePos,
      Translation2d algaeVel,
      double robotMaxAccel,
      double robotMaxVel,
      double algaeDecel,
      double maxTime,
      double dt) {

    // For storing minimum distance and its corresponding time
    double minDistance = Double.POSITIVE_INFINITY;
    double minDistanceTime = 0;
    Translation2d minDistanceRobotPos = null;
    Translation2d minDistanceAlgaePos = null;

    // Convert to mutable copies for simulation
    double robotPosX = robotPos.getX();
    double robotPosY = robotPos.getY();
    double robotVelX = robotVel.getX();
    double robotVelY = robotVel.getY();
    double algaePosX = algaePos.getX();
    double algaePosY = algaePos.getY();
    double algaeVelX = algaeVel.getX();
    double algaeVelY = algaeVel.getY();

    // Simulate motion over time using the exact same physics as the main simulation
    double t = 0;
    while (t < maxTime) {
      // Current distance between robot and algae
      double dx = robotPosX - algaePosX;
      double dy = robotPosY - algaePosY;
      double currentDistance = Math.hypot(dx, dy);

      // Check if this is the closest they've been
      if (currentDistance < minDistance) {
        minDistance = currentDistance;
        minDistanceTime = t;
        minDistanceRobotPos = new Translation2d(robotPosX, robotPosY);
        minDistanceAlgaePos = new Translation2d(algaePosX, algaePosY);
      }

      // Check for interception
      if (currentDistance < COLLISION_THRESHOLD) {
        // Midpoint between them
        Translation2d interceptPoint = new Translation2d(
            (robotPosX + algaePosX) / 2,
            (robotPosY + algaePosY) / 2);
        return new InterceptionResult(interceptPoint, t);
      }

      // Calculate direction to algae
      dx = algaePosX - robotPosX;
      dy = algaePosY - robotPosY;
      double distance = Math.hypot(dx, dy);

      // Normalized direction vector
      double dirX = 0;
      double dirY = 0;
      if (distance > 0) {
        dirX = dx / distance;
        dirY = dy / distance;
      }

      // Update robot velocity with acceleration toward algae
      robotVelX += dirX * robotMaxAccel * dt;
      robotVelY += dirY * robotMaxAccel * dt;

      // Enforce max velocity constraint
      double speed = Math.hypot(robotVelX, robotVelY);
      if (speed > robotMaxVel) {
        robotVelX = (robotVelX / speed) * robotMaxVel;
        robotVelY = (robotVelY / speed) * robotMaxVel;
      }

      // Update robot position
      robotPosX += robotVelX * dt;
      robotPosY += robotVelY * dt;

      // Update algae velocity (with deceleration if applicable)
      double algaeSpeed = Math.hypot(algaeVelX, algaeVelY);
      if (algaeSpeed > 0 && algaeDecel > 0) {
        // Direction of algae motion
        double dirAlgaeX = algaeVelX / algaeSpeed;
        double dirAlgaeY = algaeVelY / algaeSpeed;

        // Apply deceleration (capped to prevent velocity reversal)
        double decelMagnitude = Math.min(algaeDecel * dt, algaeSpeed);
        algaeVelX -= dirAlgaeX * decelMagnitude;
        algaeVelY -= dirAlgaeY * decelMagnitude;
      }

      // Update algae position
      algaePosX += algaeVelX * dt;
      algaePosY += algaeVelY * dt;

      // Advance time
      t += dt;
    }

    // If no direct interception was found, return closest approach if it's close
    // enough
    if (minDistance < COLLISION_THRESHOLD * 1.5) {
      Translation2d interceptPoint = new Translation2d(
          (minDistanceRobotPos.getX() + minDistanceAlgaePos.getX()) / 2,
          (minDistanceRobotPos.getY() + minDistanceAlgaePos.getY()) / 2);
      return new InterceptionResult(interceptPoint, minDistanceTime);
    }

    // No interception possible
    return new InterceptionResult(null, 0);
  }

  /**
   * Wrapper class to hold interception calculation results.
   */
  public class InterceptionResult {
    public final Translation2d point;
    public final double time;

    public InterceptionResult(Translation2d point, double time) {
      this.point = point;
      this.time = time;
    }
  }

  /**
   * Calculates the predicted interception point with the algae using the current
   * robot and algae states. This is a convenience method that uses the current
   * robot
   * and algae poses and velocities.
   * 
   * @param robotVel      The current velocity of the robot (in m/s)
   * @param algaeVel      The current velocity of the algae (in m/s)
   * @param robotMaxAccel The maximum acceleration of the robot (in m/s²)
   * @param robotMaxVel   The maximum velocity of the robot (in m/s)
   * @return The predicted interception point as a Translation2d, or null if
   *         interception is not possible
   */
  public Translation2d getPredictedInterceptionPoint(
      Translation2d robotVel,
      Translation2d algaeVel,
      double robotMaxAccel,
      double robotMaxVel) {

    // Use current poses from the subsystem
    Pose2d robotPose = drivetrainPoseSupplier.get();
    Pose2d algaePose = getRelativeToField();

    // Default values
    double algaeDecel = this.algaeDecel;

    // Call the full calculation method
    InterceptionResult result = calculateInterception(
        robotPose,
        robotVel,
        algaePose,
        algaeVel,
        robotMaxAccel,
        robotMaxVel,
        algaeDecel,
        Constants.Vision.AlgaeInterceptionConstants.maxTime,
        Constants.Vision.AlgaeInterceptionConstants.dt);

    return result.point;
  }
}