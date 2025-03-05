// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8; // Default 4.8 - Max net robot translational speed
    public static final double kMaxWheelSpeedMetersPerSecond = 4.8; // Max possible speed for wheel
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kBalleyPopMetersPerSecond = 0.8; // Max net robot translational speed when intaking algae
                                                                // stacked on coral

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kDriverControllerXAxis = 0;
    public static final int kDriverControllerYAxis = 1;
    public static final int kDriverControllerRotAxis = 4;
    public static final int kDriverControllerCalibrateButton = 1;

    public static final int kOperatorControllerPort = 1;
    public static final double kArmDeadband = 0.1;
    public static final int kOperatorArmManualAxis = 1;
    public static final double kIntakeDeadband = 0.5;
    public static final int kOperatorIntakeManualAxis = 5;
  }

  public static final class AutoConstants {
    // Auto Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 2; // Default 4.8
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kPTranslationController = 1.5;
    public static final double kPThetaController = 1;
    public static final double kITranslationController = 0.75;
    public static final double kIThetaController = 0;
    public static final double kDTranslationController = 0.25;
    public static final double kDThetaController = 0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ArmConstants {
    public static final int kMotorCanId = 10;
    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final double kPUpPositionController = 1.5;
    public static final double kIUpPositionController = 0;
    public static final double kDUpPositionController = 0;
    public static final double kUpPositionFF = 0;

    public static final double kPDownPositionController = 1.5;
    public static final double kIDownPositionController = 0;
    public static final double kDDownPositionController = 0;
    public static final double kDownPositionFF = 0;

    public static final double kGearReduction = 1.0 / 50; // Gear ratio

    public static final double kAngleConversionFactor = 2 * Math.PI / 360; // Degrees to radians
    public static final double kPositionConversionFactor = 2 * Math.PI * kGearReduction; // Rotations to radians
    public static final double kVelocityConversionFactor = 2 * Math.PI / 60 * kGearReduction; // RPM to radians/sec

    // Max velocity of arm in RPM for manual joystick control
    public static final double kMaxVelocity = 60;

    // Tolerance of arm position PID in degrees
    public static final double kAngleTolerance = 15;

    // Angles need to be set in degrees
    public static final double kAngleStowed = -6.5;
    public static final double kAngleGroundIntake = -60.5;
    public static final double kAngleCoral = -98.5;
    public static final double kAngleL2 = -88.5;
    public static final double kAngleL3 = -150.5;
    public static final double kAngleProcessor = -186.5;

    // Limit switch stuff
    public static final int kLimitSwitchChannel = 9;
  }

  public static final class IntakeConstants {
    public static final int kMotorCanId = 9;
    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final double kMotorReduction = 1 / 10.0;

    public static final double kPositionConversionFactor = (2 * Math.PI); // Rotations to radians
    public static final double kVelocityConversionFactor = (2 * Math.PI / 60); // RPM to radians/sec

    public static final double kMaxVelocity = 60; // Max velocity of intake in RPM, used as a reference velocity

    // Percent output for intake rollers
    public static final double kDefaultPercent = 0.5;

    // Percent output for algae intake rollers
    public static final double kDefaultAlgaeIntake = 0.7;
  }

  public static final class LimelightConstants {
    public static final String kNetworkTableKey = "limelight";
    public static final String kNetworkTableEntry = "llpython";
    public static final boolean hasBorders = true;
  }

  public static final class Vision {
    public static final class AlgaeConstants {
      // public static final String kNetworkTableKey = "algae_vision";
      public static final String kNetworkTableKey = "photonvision";
      public static final int kUpperBound = 2;
      public static final int kLowerBound = -2;
    }

    public static final class PhotonConstants {
      public static final double camera1Roll = 0;
      public static final double camera1Pitch = -10 * Math.PI / 180; // in rad
      public static final double camera1Yaw = 0;
      public static final double camera1X = 0.2921; // forward (pos)
      public static final double camera1Y = 0.127; // left (pos)
      public static final double camera1Z = 0.4699; // up (pos)

      public static final double camera2Roll = 0;
      public static final double camera2Pitch = -10 * Math.PI / 180; // in rad
      public static final double camera2Yaw = 0;
      public static final double camera2X = -0.2921;
      public static final double camera2Y = 0;
      public static final double camera2Z = 0.3175;

      public static final int kAlgaePipeline = 1;
      public static final int kAprilTagPipeline = 0;

      public static final String kCamera1 = "camera1";
      public static final String kCamera2 = "camera2";

      public static Transform3d robotToCamera = new Transform3d(camera1X, camera1Y, camera1Z,
          new Rotation3d(camera1Roll, camera1Pitch, camera1Yaw));
    }
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class IndexerConstants {
    public static final int kMotorCanId = 11;
    public static final MotorType kMotorType = MotorType.kBrushless;

    /**
     * Multiplier at which decides whether + or - inputs move the algae towards the
     * launcher.
     */
    public static final double kDirectionConstant = -1.0;

    public static final double kMotorReduction = 1.0;
    public static final double kWheelDiameterMeters = 0.17;

    // Ports for sensors. TBD
    public static final int kIntakeSensorChannel = 0;
    public static final int kLauncherSensorChannel = 1;
  }

  public static final class LauncherConstants {
    public static final int kFrontCanId = 12;
    public static final int kBackCanId = 13;
    public static MotorType kMotorType = MotorType.kBrushless;

    public static final double kPVelocityController = 0;
    public static final double kIVelocityController = 0;
    public static final double kDVelocityController = 0;
    public static final double frontKVelocityFF = 0.0017;
    public static final double backKVelocityFF = 0.00187;

    public static final double kVelocityConversionFactor = 2.0 * Math.PI / 60.0; // RPM to radians/sec

    // Velocities in RPM
    public static final double kVelocityFront = 2196.338215; // 230 rad/s
    public static final double kVelocityBack = 2721.549527; // 285 rad/s
    public static final double kVelocityFrontTolerance = 5;
    public static final double kVelocityBackTolerance = 5;
  }

  // PDP CAN IDs
  public static final int kPDPCanId = 0;
}