// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.LaunchingDistances;
import frc.robot.Constants.Vision.PhotonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LauncherConstants;

/**
 * The Drivetrain class represents the robot's drivetrain subsystem.
 * It extends the SubsystemBase class and manages the swerve drive modules and
 * odometry.
 */
public class Drivetrain extends SubsystemBase {
  // Array to hold the four swerve drive modules (front-left, front-right,
  // back-left, back-right)
  private final MAXSwerveModule[] modules = new MAXSwerveModule[4]; // FL, FR, BL, BR

  // Gyro sensor to get the robot's orientation
  public final Gyro gyro;
  public boolean gyroDisconnected;
  public boolean hasVisionData = false;
  public boolean isFieldRelative = true;
  Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  // Current pose of the robot
  Pose2d pose, visionlessPose;

  // Cameras & Photonvision variables
  private final PhotonCamera camera1;
  private final PhotonCamera camera2;

  // Photonvision -> Guess where the robot is on the field
  private final PhotonPoseEstimator photonPoseEstimator1;
  private final PhotonPoseEstimator photonPoseEstimator2;

  // Odometry class for tracking the robot's position on the field
  SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      new Rotation2d(),
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      },
      new Pose2d(0, 0, new Rotation2d()));

  SwerveDrivePoseEstimator visionlessOdometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      new Rotation2d(),
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      },
      new Pose2d(0, 0, new Rotation2d()));

  private final Field2d dashField = new Field2d();

  /**
   * Constructor for the Drivetrain class.
   * Initializes the swerve modules and gyro sensor.
   * 
   * @param moduleFL Front-left swerve module
   * @param moduleFR Front-right swerve module
   * @param moduleBL Back-left swerve module
   * @param moduleBR Back-right swerve module
   * @param gyro     Gyro sensor
   */
  public Drivetrain(MAXSwerveModule moduleFL, MAXSwerveModule moduleFR, MAXSwerveModule moduleBL,
      MAXSwerveModule moduleBR, Gyro gyro) {
    modules[0] = moduleFL;
    modules[1] = moduleFR;
    modules[2] = moduleBL;
    modules[3] = moduleBR;
    this.gyro = gyro;

    // PathPlanner Configs. These should be
    // automatically generated by the PathPlanner GUI
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner config", e);
    }

    // Set the robot's parameters for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setOdometry,
        this::getChassisSpeedsRobotRelative,
        (speeds, feedforwards) -> runVelocityRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPTranslationController, AutoConstants.kITranslationController,
                AutoConstants.kDTranslationController),
            new PIDConstants(AutoConstants.kPThetaController, AutoConstants.kIThetaController,
                AutoConstants.kDThetaController)),
        config,
        () -> {
          Optional<Alliance> alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false; // default to blue
        },
        this);

    // Setup cameras to see april tags. Wow! That makes me really happy.
    camera1 = new PhotonCamera(PhotonConstants.kCamera1);
    camera2 = new PhotonCamera(PhotonConstants.kCamera2);
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    Transform3d robotToCam1 = new Transform3d(
        new Translation3d(PhotonConstants.camera1X, PhotonConstants.camera1Y,
            PhotonConstants.camera1Z),
        new Rotation3d(PhotonConstants.camera1Roll, PhotonConstants.camera1Pitch,
            PhotonConstants.camera1Yaw));
    Transform3d robotToCam2 = new Transform3d(
        new Translation3d(PhotonConstants.camera2X, PhotonConstants.camera2Y,
            PhotonConstants.camera2Z),
        new Rotation3d(PhotonConstants.camera2Roll, PhotonConstants.camera2Pitch,
            PhotonConstants.camera2Yaw));

    photonPoseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam1);
    photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam2);
  }

  /**
   * This method is called periodically by the scheduler.
   * It updates the robot's pose using the odometry class.
   */
  @Override
  public void periodic() {
    // Get the current angle from the gyro sensor
    var gyroAngle = gyro.getRotation2d();
    if (!m_debouncer.calculate(gyro.isConnected())) {
      gyroDisconnected = true;
      isFieldRelative = false;
    }

    // Update the robot's pose using the odometry class
    if (!gyroDisconnected) {
      SwerveModulePosition[] positions = new SwerveModulePosition[] {
          modules[0].getPosition(), modules[1].getPosition(),
          modules[2].getPosition(), modules[3].getPosition()
      };
      pose = odometry.update(gyroAngle, positions);
      visionlessPose = visionlessOdometry.update(gyroAngle, positions);
    }

    Optional<EstimatedRobotPose> visionPose1 = getEstimatedGlobalPose1(pose);
    Optional<EstimatedRobotPose> visionPose2 = getEstimatedGlobalPose2(pose);

    hasVisionData = false;
    List<Integer> fiducialIds1 = new ArrayList<>();
    List<Integer> fiducialIds2 = new ArrayList<>();
    if (visionPose1.isPresent()) {
      Logger.recordOutput("Vision/Photon1/EstimatedPose", visionPose1.get().estimatedPose);

      // Add targets to list
      for (PhotonTrackedTarget target : visionPose1.get().targetsUsed) {
        fiducialIds1.add(target.fiducialId);
      }

      Pose3d visionPose = visionPose1.get().estimatedPose;
      Pose2d estimatedPose = new Pose2d(visionPose.getX(), visionPose.getY(),
          new Rotation2d(visionPose.getRotation().getZ()));
      odometry.addVisionMeasurement(estimatedPose, visionPose1.get().timestampSeconds);
      Logger.recordOutput("Vision/Photon1/Timestamp", visionPose1.get().timestampSeconds);
      hasVisionData = true;
    } else {
      Logger.recordOutput("Vision/Photon1/EstimatedPose", new Pose3d());
      Logger.recordOutput("Vision/Photon1/Timestamp", -1.0);
    }
    if (visionPose2.isPresent()) {
      Logger.recordOutput("Vision/Photon2/EstimatedPose", visionPose2.get().estimatedPose);

      // Add targets to list
      for (PhotonTrackedTarget target : visionPose2.get().targetsUsed) {
        fiducialIds2.add(target.fiducialId);
      }

      Pose3d visionPose = visionPose2.get().estimatedPose;
      Pose2d estimatedPose = new Pose2d(visionPose.getX(), visionPose.getY(),
          new Rotation2d(visionPose.getRotation().getZ()));
      odometry.addVisionMeasurement(estimatedPose, visionPose2.get().timestampSeconds);
      Logger.recordOutput("Vision/Photon2/Timestamp", visionPose2.get().timestampSeconds);
      hasVisionData = true;
    } else {
      Logger.recordOutput("Vision/Photon2/EstimatedPose", new Pose3d());
      Logger.recordOutput("Vision/Photon2/Timestamp", -1.0);
    }
    SmartDashboard.putBoolean("HasVision", hasVisionData);

    // Log april tags to the logger
    Logger.recordOutput("Vision/Photon1/TargetIds", fiducialIds1.toString());
    Logger.recordOutput("Vision/Photon2/TargetIds", fiducialIds2.toString());

    // Log the current pose to the logger
    Logger.recordOutput("Odometry/WithVisionInput", pose);
    Logger.recordOutput("Odometry/WithoutVisionInput", visionlessPose);
    SmartDashboard.putData("Odometry", dashField);
    dashField.setRobotPose(pose);

    // Create arrays to hold the states and positions of the swerve modules
    SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // Populate the arrays with the current states and positions of the swerve
    // modules
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
      positions[i] = modules[i].getPosition();
    }

    // Log the states and positions of the swerve modules to the logger
    Logger.recordOutput("Swerve/Module/State", states);
    Logger.recordOutput("Swerve/Module/Position", positions);

    // Distance to barge
    double distanceToBarge = distanceToBardge();
    double xDistanceToBarge = getXDistanceToBarge();

    SmartDashboard.putNumber("Distance to Barge", distanceToBarge);
    SmartDashboard.putNumber("Distance to Barge (x)", xDistanceToBarge);

    SmartDashboard.putBoolean("FarFromBargeLaunchingRange",
        xDistanceToBarge > LaunchingDistances.kMetersFromBarge);
    SmartDashboard.putBoolean("WithinBargeLaunchingRange",
        MathUtil.isNear(xDistanceToBarge, LaunchingDistances.kMetersFromBarge,
            LaunchingDistances.kToleranceMetersFromBarge));

    Logger.recordOutput("Vision/Bardge/DistanceToBardge", distanceToBarge);
    Logger.recordOutput("Vision/Bardge/DistanceToBargeX", xDistanceToBarge);

    Logger.recordOutput("Drive/FieldRelative", isFieldRelative);

    Logger.recordOutput("Drive/GyroDisconnected", gyroDisconnected);
    SmartDashboard.putBoolean("GyroDisconnected", gyroDisconnected);
  }

  /**
   * Runs the drivetrain at the specified velocities relative to the field.
   * 
   * @param speeds The desired chassis speeds
   */
  public void runVelocityFieldRelative(ChassisSpeeds speeds) {
    runVelocity(speeds, true);
  }

  /**
   * Runs the drivetrain at the specified velocities relative to the robot.
   * 
   * @param speeds The desired chassis speeds
   */
  public void runVelocityRobotRelative(ChassisSpeeds speeds) {
    runVelocity(speeds, false);
  }

  /**
   * Runs the drivetrain at the specified velocities.
   * 
   * @param speeds          The desired chassis speeds
   * @param isFieldRelative Whether the speeds are relative to the field
   */
  public void runVelocity(ChassisSpeeds speeds, Boolean isFieldRelative) {
    // If the speeds are field-relative, convert them to robot-relative speeds
    if (isFieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          gyro.getRotation2d());
    }
    // Convert the chassis speeds to swerve module states
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Desaturate the wheel speeds to ensure they are within the maximum speed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxWheelSpeedMetersPerSecond);

    // Set the desired state for each swerve module
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }

    // Log the desired states of the swerve modules to the logger
    Logger.recordOutput("Swerve/Module/Setpoint", states);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, isFieldRelative);
  }

  /**
   * Gets the current state of each swerve module.
   * 
   * @return An array of SwerveModuleState representing the state of each swerve
   *         module.
   */
  public SwerveModuleState[] getModuleState() {
    return new SwerveModuleState[] {
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState()
    };
  }

  /**
   * Gets the current position of each swerve module.
   * 
   * @return An array of SwerveModulePosition representing the position of each
   *         swerve module.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
    };
  }

  /**
   * Gets the current chassis speeds relative to the robot.
   * 
   * @return A ChassisSpeeds object representing the robot-relative chassis
   *         speeds.
   */
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState());
  }

  public void setOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    visionlessOdometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometry() {
    setOdometry(new Pose2d(
        new Translation2d(0, 0),
        new Rotation2d()));
  }

  // These are a couple of methods that the AutoBuilder relies on!
  /**
   * Gets the current pose of the robot.
   * 
   * @return A Pose2d object representing the current pose of the robot.
   */
  public Pose2d getPose() {
    return pose;
  }

  public ChassisSpeeds getChassisSpeedsRobotRelative() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState());
  }

  public Pose2d convertPose3d(Pose3d p) {
    double x = p.getX();
    double y = p.getY();
    Rotation2d rot = new Rotation2d(p.getRotation().getAngle());
    return new Pose2d(x, y, rot);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose1(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator1.update(camera1.getLatestResult());
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose2(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator2.update(camera2.getLatestResult());
  }

  public boolean isOnBlueSide() {
    double robotX = getPose().getTranslation().getX();
    return robotX - FieldConstants.kBargeX < 0;
  }

  public double getXDistanceToBarge() {
    double robotX = getPose().getTranslation().getX();
    return Math.abs(robotX - FieldConstants.kBargeX);
  }

  public double distanceToBardge() {
    double robotYaw = getPose().getRotation().getRadians();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      robotYaw = robotYaw + Math.PI;
    }
    return robotYaw > -Math.PI / 2 && robotYaw < Math.PI / 2 ? (getXDistanceToBarge() / Math.cos(robotYaw)) : -1;
  }

  public Command moveToNearestBargeLaunchingZone() {
    Pose2d startingPose = getPose();
    boolean onBlueSide = isOnBlueSide();
    double targetRadians;
    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      targetRadians = (onBlueSide == (alliance == Alliance.Blue)) ? 0 : Math.PI;
    } else {
      targetRadians = startingPose.getRotation().getRadians();
    }
    Pose2d targetPose = new Pose2d(
        FieldConstants.kBargeX + ((onBlueSide ? -1 : 1) * LaunchingDistances.kMetersFromBarge), startingPose.getY(),
        new Rotation2d(targetRadians));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startingPose, targetPose);
    PathPlannerPath path = new PathPlannerPath(waypoints, DriveConstants.kPathfindingConstraints, null,
        new GoalEndState(0, Rotation2d.fromRadians(targetRadians)));
    path.preventFlipping = true;

    Logger.recordOutput("PathPlanner/GoToBarge/StartingPose", startingPose);
    Logger.recordOutput("PathPlanner/GoToBarge/TargetPose", targetPose);

    return AutoBuilder.followPath(path);
  }
}