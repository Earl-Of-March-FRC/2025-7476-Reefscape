// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.LaunchingDistances;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Vision.PhotonConstants;
import frc.utils.PoseHelpers;

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
  public Supplier<Boolean> isUsingHighVelocities = () -> true;
  Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  // Current pose of the robot
  Pose2d pose = new Pose2d();
  Pose2d visionlessPose = new Pose2d();

  // Cameras & Photonvision variables
  private final PhotonCamera[] cameras = new PhotonCamera[PhotonConstants.numCameras];

  // Photonvision -> Guess where the robot is on the field
  private final PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[PhotonConstants.numCameras];

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
      new Pose2d(0, 0, new Rotation2d()),
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.4, 0.4, 0.4));

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
   * @param moduleFL              Front-left swerve module
   * @param moduleFR              Front-right swerve module
   * @param moduleBL              Back-left swerve module
   * @param moduleBR              Back-right swerve module
   * @param gyro                  Gyro sensor
   * @param isUsingHighVelocities If the robot is launching with high velocities
   */
  public Drivetrain(MAXSwerveModule moduleFL, MAXSwerveModule moduleFR, MAXSwerveModule moduleBL,
      MAXSwerveModule moduleBR, Gyro gyro, Supplier<Boolean> isUsingHighVelocities) {
    this(moduleFL, moduleFR, moduleBL, moduleBR, gyro);
    this.isUsingHighVelocities = isUsingHighVelocities;
  }

  /**
   * Constructor for the Drivetrain class.
   * Initializes the swerve modules and gyro sensor.
   * Assumes the robot is launching with high velocities.
   * See
   * {@link #Drivetrain(MAXSwerveModule, MAXSwerveModule, MAXSwerveModule, MAXSwerveModule, Gyro, Supplier)}
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
    for (int i = 0; i < PhotonConstants.numCameras; i++) {
      cameras[i] = new PhotonCamera(PhotonConstants.kCameras[i]);
      photonPoseEstimators[i] = new PhotonPoseEstimator(FieldConstants.kfieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          PhotonConstants.kRobotToCams[i]);
    }

    // Log april tag poses to logger
    FieldConstants.kfieldLayout.getTags()
        .forEach((tag) -> Logger.recordOutput("FieldLayout/AprilTags/" + tag.ID, tag.pose));
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

    hasVisionData = false;

    for (int i = 0; i < PhotonConstants.numCameras; i++) {
      List<EstimatedRobotPose> visionPoses = getEstimatedGlobalPose(photonPoseEstimators[i], cameras[i],
          PhotonConstants.kRobotToCams[i],
          pose);

      List<Integer> fiducialIds = new ArrayList<>();
      List<Pose3d> fiducialIdPoses = new ArrayList<>();
      List<Double> tagAreas = new ArrayList<>();
      List<Double> tagAmbiguities = new ArrayList<>();
      for (EstimatedRobotPose visionPose : visionPoses) {
        Logger.recordOutput("Vision/" + cameras[i].getName() + "/EstimatedPose", visionPose.estimatedPose);
        Logger.recordOutput("Vision/" + cameras[i].getName() + "/Timestamp", visionPose.timestampSeconds);

        // Add targets to list
        for (PhotonTrackedTarget target : visionPose.targetsUsed) {
          fiducialIds.add(target.fiducialId);
          if (FieldConstants.kfieldLayout.getTagPose(target.fiducialId).isPresent()) {
            fiducialIdPoses.add(FieldConstants.kfieldLayout.getTagPose(target.fiducialId).get());
          }
          tagAreas.add(target.area);
          tagAmbiguities.add(target.poseAmbiguity);

          // if (target.fiducialId == 10) {
          // Transform3d camToTarget = target.bestCameraToTarget;
          // Transform3d robotToTag = new Transform3d(0.825, 0, 0.5715, new Rotation3d(0,
          // 0,
          // Math.PI));
          // // Transform3d robotToTag = new Transform3d(2, 0, 0.5715, new Rotation3d());
          // // Pose3d robotToCam = TagUtils.getRobotPose3dFromCamera(camToTarget,
          // // robotToTag);
          // Transform3d robotToCam = TagUtils.getRobotPose3dFromCamera(camToTarget,
          // robotToTag);
          // Logger.recordOutput("Vision/" + cameras[i].getName() + "/TagUtil",
          // robotToCam);
          // }
        }

        Logger.recordOutput("Vision/" + cameras[i].getName() + "/TargetIds",
            fiducialIds.stream().mapToInt(n -> n).toArray());
        Logger.recordOutput("Vision/" + cameras[i].getName() + "/TargetPoses",
            fiducialIdPoses.toArray(new Pose3d[fiducialIdPoses.size()]));
        Logger.recordOutput("Vision/" + cameras[i].getName() + "/TargetAreas",
            tagAreas.stream().mapToDouble(n -> n).toArray());
        Logger.recordOutput("Vision/" + cameras[i].getName() + "/TargetAmbiguities",
            tagAmbiguities.stream().mapToDouble(n -> n).toArray());

        Pose2d estimatedPose = PoseHelpers.toPose2d(visionPose.estimatedPose);
        Vector<N3> standardDeviation = PhotonConstants.kCameraStandardDeviations.get(i);

        // if (i != 1) {
        odometry.setVisionMeasurementStdDevs(standardDeviation);
        odometry.addVisionMeasurement(estimatedPose, visionPose.timestampSeconds);
        hasVisionData = true;
        // }

      }
      if (visionPoses.isEmpty()) {
        Logger.recordOutput("Vision/" + cameras[i].getName() + "/EstimatedPose",
            new Pose3d(-1, -1, -1, new Rotation3d()));
        Logger.recordOutput("Vision/" + cameras[i].getName() + "/Timestamp", -1.0);
        Logger.recordOutput("Vision/" + cameras[i].getName() + "/TargetIds", new int[0]);
        Logger.recordOutput("Vision/" + cameras[i].getName() + "/TargetPoses", new Pose3d[0]);
      }
    }

    SmartDashboard.putBoolean("HasVision", hasVisionData);

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
        xDistanceToBarge > (isUsingHighVelocities.get() ? LaunchingDistances.kMetersFromBargeHigh
            : LaunchingDistances.kMetersFromBargeLow));
    SmartDashboard.putBoolean("WithinBargeLaunchingRange", MathUtil.isNear(xDistanceToBarge,
        (isUsingHighVelocities.get() ? LaunchingDistances.kMetersFromBargeHigh
            : LaunchingDistances.kMetersFromBargeLow),
        LaunchingDistances.kToleranceMetersFromBarge));

    Logger.recordOutput("Vision/Bardge/DistanceToBardge", distanceToBarge);
    Logger.recordOutput("Vision/Bardge/DistanceToBargeX", xDistanceToBarge);

    Logger.recordOutput("Drive/FieldRelative", isFieldRelative);

    Logger.recordOutput("Drive/GyroDisconnected", gyroDisconnected);
    SmartDashboard.putBoolean("GyroDisconnected", gyroDisconnected);

    // Log which side the robot is on
    Logger.recordOutput("Odometry/IsOnBlueSide", PoseHelpers.isOnBlueSide(getPose()));
  }

  /**
   * Runs the drivetrain at the specified velocities relative to the field.
   * 
   * @see #runVelocity(ChassisSpeeds, Boolean)
   * @see #runVelocityRobotRelative(ChassisSpeeds)
   * 
   * @param speeds The desired field relative chassis speeds
   */
  public void runVelocityFieldRelative(ChassisSpeeds speeds) {
    runVelocity(speeds, true);
  }

  /**
   * Runs the drivetrain at the specified velocities relative to the robot.
   * 
   * @see #runVelocity(ChassisSpeeds, Boolean)
   * @see #runVelocityFieldRelative(ChassisSpeeds)
   * 
   * @param speeds The desired robot relative chassis speeds
   */
  public void runVelocityRobotRelative(ChassisSpeeds speeds) {
    runVelocity(speeds, false);
  }

  /**
   * Runs the drivetrain at the specified velocities.
   * 
   * @see #runVelocityFieldRelative(ChassisSpeeds)
   * @see #runVelocityRobotRelative(ChassisSpeeds)
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
   * Sets the current pose of the robot into odometry.
   * 
   * @param pose A Pose2d object representing the current pose of the robot.
   */
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

  /**
   * Gets the current chassis speeds relative to the robot.
   * 
   * @return A ChassisSpeeds object representing the robot-relative chassis
   *         speeds.
   */
  public ChassisSpeeds getChassisSpeedsRobotRelative() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState());
  }

  /**
   * Gets the current chassis speeds relative to the field.
   * 
   * @return A ChassisSpeeds object representing the field-relative chassis
   *         speeds.
   */
  public ChassisSpeeds getChassisSpeedsFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeedsRobotRelative(),
        odometry.getEstimatedPosition().getRotation());
  }

  /**
   * Get an estimate of the robot's pose using vision data from PhotonVisision.
   * This method will filter out invalid and unprobable results.
   * 
   * @param poseEstimator The PhotonPoseEstimator object to use for
   *                      estimating the robot's pose
   * @param camera        The PhotonCamera object to use for getting vision data
   * @param robotToCam    The Transform3d object representing the transformation
   *                      from the robot to the camera
   */
  public List<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator poseEstimator, PhotonCamera camera,
      Transform3d robotToCam,
      Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);

    List<EstimatedRobotPose> results = new ArrayList<>();
    List<PhotonPipelineResult> camResults = camera.getAllUnreadResults();

    for (PhotonPipelineResult camResult : camResults) {
      if (!camResult.hasTargets()) {
        continue;
      }

      // check if the built in pose estimator pose is reasonable
      Optional<EstimatedRobotPose> optionalEstimation = poseEstimator.update(camResult);
      if (optionalEstimation.isPresent()) {
        EstimatedRobotPose estimation = optionalEstimation.get();

        Logger.recordOutput("Vision/" + camera.getName() + "/RawEstimatedPose", estimation.estimatedPose);

        if (PoseHelpers.isInField(estimation.estimatedPose) &&
            PoseHelpers.isOnGround(estimation.estimatedPose, PhotonConstants.kHeightTolerance)) {

          // ignore the result if it only has one tag and the tag is too small
          if (camResult.getTargets().size() == 1
              && camResult.getTargets().get(0).area <= PhotonConstants.kMinSingleTagArea) {
            continue;
          }

          results.add(estimation);
          continue;
        }
      }

      double timestamp = camResult.getTimestampSeconds();
      List<PhotonTrackedTarget> targetsUsed = new ArrayList<>();

      // if the built in pose estimator is not reasonable, compute it ourselves
      if (camResult.hasTargets()) {
        List<PhotonTrackedTarget> targets = camResult.getTargets();
        List<Pose3d> validPoses = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {

          // ignore targets with high pose ambiguity
          if (target.getPoseAmbiguity() > PhotonConstants.kAmbiguityDiscardThreshold) {
            continue;
          }

          int targetId = target.fiducialId;
          Optional<Pose3d> optionalTagPose = FieldConstants.kfieldLayout.getTagPose(targetId);
          // it should never be empty, but just in case
          if (optionalTagPose.isEmpty()) {
            continue;
          }
          Pose3d tagPose = optionalTagPose.get();

          // if the ambiguity is high, only use the pose that is reasonable
          if (target.getPoseAmbiguity() > PhotonConstants.kAmbiguityThreshold) {
            Transform3d bestCamToTarget = target.getBestCameraToTarget();
            Transform3d altCamToTarget = target.getAlternateCameraToTarget();

            // robotTransform = tagTransform - camToTarget - robotToCam
            Pose3d bestRobotPose = tagPose.transformBy(bestCamToTarget.inverse())
                .transformBy(robotToCam.inverse());
            Pose3d altRobotPose = tagPose.transformBy(altCamToTarget.inverse()).transformBy(robotToCam.inverse());

            Logger.recordOutput("Vision/" + camera.getName() + "/FallbackBestPose", bestRobotPose);
            Logger.recordOutput("Vision/" + camera.getName() + "/FallbackAltPose", altRobotPose);

            // check if they are reasonable
            boolean isBestPoseValid = PoseHelpers.isInField(bestRobotPose) &&
                PoseHelpers.isOnGround(bestRobotPose, PhotonConstants.kHeightTolerance);
            boolean isAltPoseValid = PoseHelpers.isInField(altRobotPose)
                && PoseHelpers.isOnGround(altRobotPose, PhotonConstants.kHeightTolerance);
            if (isBestPoseValid && isAltPoseValid) {
              targetsUsed.add(target);
              // if both are valid, use the one that is closer to the previous estimation
              double bestDistance = PoseHelpers.distanceBetween(bestRobotPose, new Pose3d(prevEstimatedRobotPose));
              double altDistance = PoseHelpers.distanceBetween(altRobotPose, new Pose3d(prevEstimatedRobotPose));
              if (bestDistance < altDistance) {
                validPoses.add(bestRobotPose);
              } else {
                validPoses.add(altRobotPose);
              }
            } else if (isBestPoseValid) {
              targetsUsed.add(target);
              validPoses.add(bestRobotPose);
            } else if (isAltPoseValid) {
              targetsUsed.add(target);
              validPoses.add(altRobotPose);
            }
            continue;
          }

          // if the ambiguity is low, use the pose directly
          Transform3d camToTarget = target.getBestCameraToTarget();
          // robotTransform = tagTransform - camToTarget - robotToCam
          Pose3d robotPose = tagPose.transformBy(camToTarget.inverse()).transformBy(robotToCam.inverse());
          // check if the pose is reasonable
          if (PoseHelpers.isInField(robotPose) && PoseHelpers.isOnGround(robotPose, PhotonConstants.kHeightTolerance)) {
            validPoses.add(robotPose);
            targetsUsed.add(target);
          }
        }

        // if there are no valid poses, ignore this frame
        if (validPoses.isEmpty()) {
          continue;
        }

        double totalX = 0;
        double totalY = 0;
        double totalZRot = 0;

        for (Pose3d pose : validPoses) {
          totalX += pose.getX();
          totalY += pose.getY();

          totalZRot += pose.getRotation().getZ();
        }

        final int count = validPoses.size();
        Pose3d averagePose = new Pose3d(
            totalX / count, // X average
            totalY / count, // Y average
            0, // Z forced to 0 (validated by isOnGround)
            new Rotation3d(0, 0, totalZRot / count) // Average Z rotation
        );

        Logger.recordOutput("Vision/" + camera.getName() + "/FallbackPose", averagePose);

        // ignore the result if it only has one tag and the tag is too small
        if (camResult.getTargets().size() == 1
            && camResult.getTargets().get(0).area <= PhotonConstants.kMinSingleTagArea) {
          continue;
        }
        results
            .add(new EstimatedRobotPose(averagePose, timestamp, targetsUsed,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE));
      }
    }
    return results;
  }

  /**
   * Gets the distance between the robot and the barge, in a straight line.
   * 
   * @see #distanceToBardge()
   * 
   * @return The distance between the robot and the barge. In meters.
   */
  public double getXDistanceToBarge() {
    double robotX = getPose().getTranslation().getX();
    return Math.abs(robotX - FieldConstants.kBargeX);
  }

  /**
   * Gets the distance between the robot and the barge considering the robot's
   * angle. This is essentially how far the algae has to travel.
   * 
   * @see #getXDistanceToBarge()
   * 
   * @return The distance between the robot and the barge. In meters.
   */
  public double distanceToBardge() {
    double robotYaw = getPose().getRotation().getRadians();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      robotYaw = robotYaw + Math.PI;
    }
    return robotYaw > -Math.PI / 2 && robotYaw < Math.PI / 2 ? (getXDistanceToBarge() / Math.cos(robotYaw)) : -1;
  }

  public Command moveToNearestBargeLaunchingZone() {
    Pose2d startingPose = getPose();
    boolean onBlueSide = PoseHelpers.isOnBlueSide(getPose());
    double targetRadians;
    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      targetRadians = (onBlueSide == (alliance == Alliance.Blue)) ? 0 : Math.PI;
    } else {
      targetRadians = startingPose.getRotation().getRadians();
    }
    Pose2d targetPose = new Pose2d(
        FieldConstants.kBargeX
            + ((onBlueSide ? -1 : 1) * (isUsingHighVelocities.get() ? LaunchingDistances.kMetersFromBargeHigh
                : LaunchingDistances.kMetersFromBargeLow)),
        startingPose.getY(),
        new Rotation2d(targetRadians));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startingPose, targetPose);
    PathPlannerPath path = new PathPlannerPath(waypoints, DriveConstants.kPathfindingConstraints, null,
        new GoalEndState(0, Rotation2d.fromRadians(targetRadians)));
    path.preventFlipping = true;

    Logger.recordOutput("PathPlanner/GoToBarge/StartingPose", startingPose);
    Logger.recordOutput("PathPlanner/GoToBarge/TargetPose", targetPose);

    return AutoBuilder.followPath(path);
  }

  /**
   * Calculates the target pose for the robot at the nearest barge launching zone.
   * This takes in account if the launcher is set to high or low.
   * 
   * @param targetAngle Angle that robot should face in its target pose, in
   *                    radians. CCW is positive.
   * @return The calculated target pose for the robot at the barge.
   */
  public Pose2d getBargeTargetPose(double targetAngle) {
    System.out.println("getBargeTargetPose");
    Pose2d currentPose = getPose();

    // Y coordinate stays the same as current pose
    double targetY = currentPose.getY();

    boolean onBlueSide = PoseHelpers.isOnBlueSide(currentPose);

    double metersFromBarge = isUsingHighVelocities.get() ? LaunchingDistances.kMetersFromBargeHigh
        : LaunchingDistances.kMetersFromBargeLow;

    // Calculate target translation
    // (0,0) is ALWAYS on the blue alliance side
    double targetX = FieldConstants.kBargeX + ((onBlueSide ? -1 : 1) * metersFromBarge);

    // Calculate target rotation based on side of field that robot is currently on
    double targetRadians = (onBlueSide ? Math.PI : 0) + targetAngle;

    return new Pose2d(targetX, targetY, new Rotation2d(targetRadians));
  }
}
