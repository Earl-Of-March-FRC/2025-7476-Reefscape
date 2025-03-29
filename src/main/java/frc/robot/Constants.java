// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

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
    public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(4.8); // Default 4.8 - Max net robot translational
                                                                            // speed
    public static final LinearVelocity kMaxWheelSpeed = MetersPerSecond.of(4.8); // Max possible speed for wheel
    public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(2 * Math.PI); // radians per second
    public static final LinearVelocity kBalleyPopSpeed = MetersPerSecond.of(0.8); // Max net robot
                                                                                  // translational speed when
                                                                                  // intaking algae
    // stacked on coral
    public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(3);
    public static final LinearAcceleration kMaxAccelerationPathfinding = MetersPerSecondPerSecond.of(1);
    public static final AngularVelocity kMaxAngularSpeedPathfinding = RadiansPerSecond.of(Math.PI);
    public static final AngularVelocity kMaxAngularAccelerationPathfinding = RadiansPerSecond.of(Math.PI);

    public static final LinearVelocity kBangBangTranslationalVelocity = MetersPerSecond.of(2.5);
    public static final AngularVelocity kBangBangRotationalVelocity = RadiansPerSecond
        .of((2 * Math.PI) / 10);

    public static final PathConstraints kPathfindingConstraints = new PathConstraints(kMaxSpeed.in(MetersPerSecond),
        kMaxAccelerationPathfinding.in(MetersPerSecondPerSecond), kMaxAngularSpeedPathfinding.in(RadiansPerSecond),
        kMaxAngularAccelerationPathfinding.in(RadiansPerSecond));

    // Chassis configuration
    public static final Distance kTrackWidth = Inches.of(26.5);
    // Distance between centers of right and left wheels on robot
    public static final Distance kWheelBase = Inches.of(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase.div(2).in(Meter), kTrackWidth.div(2).in(Meter)),
        new Translation2d(kWheelBase.div(2).in(Meter), -kTrackWidth.div(2).in(Meter)),
        new Translation2d(-kWheelBase.div(2).in(Meter), kTrackWidth.div(2).in(Meter)),
        new Translation2d(-kWheelBase.div(2).in(Meter), -kTrackWidth.div(2).in(Meter)));

    // Angular offsets of the modules relative to the chassis in radians
    public static final Angle kFrontLeftChassisAngularOffset = Radians.of(-Math.PI / 2);
    public static final Angle kFrontRightChassisAngularOffset = Radians.of(0);
    public static final Angle kBackLeftChassisAngularOffset = Radians.of(Math.PI);
    public static final Angle kBackRightChassisAngularOffset = Radians.of(Math.PI / 2);

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

    public static class LaunchingDistances {
      public static final Distance kDistanceFromBarge = Meters.of(1.30); // 1.30 before March 20
      public static final Distance kToleranceDistanceFromBarge = Meters.of(0.1);
      public static final Angle kToleranceAngleFromBarge = Radians.of(5 * Math.PI / 180);

      public static final double kTargetBargeAngle = 45 * Math.PI / 180;
      public static final double kTargetBargeAngleLeft = kTargetBargeAngle;
      public static final double kTargetBargeAngleStraight = 0;
      public static final double kTargetBargeAngleRight = -kTargetBargeAngle;
    }

    public static class ReefConstants {
      // Tag oriented offset from center of robot. +x is in front of tag, +y is left
      // of tag)
      public static final Transform2d kOffsetFromTag = new Transform2d(new Translation2d(0.5, 0),
          new Rotation2d(Math.PI));
      public static final double kToleranceMetersFromSpot = 0.1;
      public static final double kToleranceRadiansFromSpot = 5 * Math.PI / 180;

      public static final int[] kReefTagIds = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
    }
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final AngularVelocity kDrivingMotorFreeSpeed = NeoMotorConstants.kFreeSpeed;
    public static final Distance kWheelDiameter = Inches.of(3);
    public static final Distance kWheelCircumference = kWheelDiameter.times(Math.PI);
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final AngularVelocity kDriveWheelFreeSpeed = RotationsPerSecond
        .of(
            kDrivingMotorFreeSpeed
                .times(kWheelCircumference)
                .div(kDrivingMotorReduction)
                .in(MultUnit.combine(RotationsPerSecond, Meters)));
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
    public static final double kArmManualDeadband = 0.5;
    public static final int kOperatorArmManualAxis = 1;
    public static final double kIntakeDeadband = 0.5;
    public static final int kOperatorIntakeManualAxis = 5;
  }

  public static final class AutoConstants {
    // Auto Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(2); // Default 4.8
    public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(3);
    public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(Math.PI);
    public static final AngularAcceleration kMaxAngularAcceleration = RadiansPerSecondPerSecond.of(Math.PI);

    public static final double kPTranslationController = 1.5;
    public static final double kPThetaController = 1;
    public static final double kITranslationController = 0.75;
    public static final double kIThetaController = 0;
    public static final double kDTranslationController = 0.25;
    public static final double kDThetaController = 0;

    public static final class EncoderAutoDriveConstants {
      public static final Distance kLeaveZoneDistance = Meters.of(0.5); // Distance to travel
      public static final LinearVelocity kLeaveZoneVelocity = MetersPerSecond.of(0.5); // Velocity (Meters/S) to leave
                                                                                       // zone at
    }

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed.in(RadiansPerSecond), kMaxAngularAcceleration.in(RadiansPerSecondPerSecond));

    public static final Pose2d kLaunchPoseBlue = new Pose2d(new Translation2d(7.475, 5.37),
        Rotation2d.fromDegrees(180));
    public static final Pose2d kLaunchPoseRed = new Pose2d(new Translation2d(10.075, 2.68), new Rotation2d(0));

  }

  public static final class NeoMotorConstants {
    public static final AngularVelocity kFreeSpeed = RotationsPerSecond.of(5676 / 60);
  }

  public static final class ArmConstants {
    public static final int kMotorCanId = 10;
    public static final MotorType kMotorType = MotorType.kBrushless;

    // public static final double kPUpPositionController = 1.5;
    public static final double kPPositionController = 0.5;
    public static final double kIPositionController = 0;
    public static final double kDPositionController = 0;
    public static final double kPositionFF = 0;
    public static final double kGainFF = 0.7;

    public static final Angle kGearReduction = Rotations.of(1.0 / 50); // Gear ratio

    // public static final double kAngleConversionFactor = 2 * Math.PI / 360; //
    // Degrees to radians
    public static final Angle kPositionConversionFactor = kGearReduction;
    public static final AngularVelocity kVelocityConversionFactor = kGearReduction.div(Seconds.of(60));

    // Tolerance of arm position PID in degrees
    public static final Angle kAngleTolerance = Degrees.of(3);

    // Arm starting position in radians
    public static final Angle kAngleStart = Radians.of(-3.863);

    // Angles need to be set in degrees
    public static final Angle kAngleStowed = Degrees.of(-6.5);
    public static final Angle kAngleGroundIntake = Degrees.of(-44.5); // 39 deg below horizontal
    public static final Angle kAngleCoral = Degrees.of(-83.5); // 8 deg above horizontal
    public static final Angle kAngleL2 = Degrees.of(-97.5);
    public static final Angle kAngleL3 = Degrees.of(-142.5);
    public static final Angle kAngleProcessor = Degrees.of(-176.5);

    // Arm PID fine control bump offsets
    public static final Angle kBumpOffset = Degrees.of(2);
    public static final double kMaxArmManualSpeedPercent = 0.5;

    // Limit switch stuff
    public static final int kLimitSwitchChannel = 9;

    // Color sensor
    public static final I2C.Port kColorSensorI2CPort = I2C.Port.kOnboard;
    public static final Color kAlgaeColor = Color.kTurquoise;
    public static final double kColorMatchThreshold = 0;
    public static final int kColorSensorProximityThreshold = 2000;
  }

  public static final class IntakeConstants {
    public static final int kMotorCanId = 9;
    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final Angle kMotorReduction = Rotations.of(1 / 10.0);

    public static final Angle kPositionConversionFactor = kMotorReduction;
    public static final AngularVelocity kVelocityConversionFactor = kMotorReduction.div(Seconds.of(60));

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
      public static final Angle camera1Roll = Degrees.of(0);
      public static final Angle camera1Pitch = Degrees.of(10);
      public static final Angle camera1Yaw = Degrees.of(0);
      public static final Distance camera1X = Meters.of(0.2921); // forward (pos)
      public static final Distance camera1Y = Meters.of(0.127); // left (pos)
      public static final Distance camera1Z = Meters.of(0.4699); // up (pos)

      public static final Angle camera2Roll = Degrees.of(0);
      public static final Angle camera2Pitch = Degrees.of(0);
      public static final Angle camera2Yaw = Degrees.of(180);
      public static final Distance camera2X = Meters.of(-0.2921);
      public static final Distance camera2Y = Meters.of(0);
      public static final Distance camera2Z = Meters.of(0.3175);

      public static final int kAlgaePipeline = 1;
      public static final int kAprilTagPipeline = 0;

      public static final String kCamera1 = "camera1";
      public static final String kCamera2 = "camera2";

      public static final Transform3d kRobotToCam1 = new Transform3d(
          new Translation3d(PhotonConstants.camera1X, PhotonConstants.camera1Y, PhotonConstants.camera1Z),
          new Rotation3d(PhotonConstants.camera1Roll, PhotonConstants.camera1Pitch, PhotonConstants.camera1Yaw));
      public static final Transform3d kRobotToCam2 = new Transform3d(
          new Translation3d(PhotonConstants.camera2X, PhotonConstants.camera2Y, PhotonConstants.camera2Z),
          new Rotation3d(PhotonConstants.camera2Roll, PhotonConstants.camera2Pitch, PhotonConstants.camera2Yaw));

      public static final Distance kHeightTolerance = Meters.of(0.5); // meters above and below ground
      public static final double kAmbiguityDiscardThreshold = 0.7; // ignore targets above this value
      public static final double kAmbiguityThreshold = 0.3; // targets above this need to be checked
      public static final double kMinSingleTagArea = 0.3;
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
    public static final Distance kWheelDiameter = Meters.of(0.17);

    // Ports for sensors. TBD
    public static final int kIntakeSensorChannel = 0;
    public static final int kLauncherSensorChannel = 1;
  }

  public static final class LauncherConstants {
    public static final int kFrontCanId = 12;
    public static final int kBackCanId = 13;
    public static MotorType kMotorType = MotorType.kBrushless;

    public static final double kPVelocityController = 0.003;
    public static final double kIVelocityController = 0;
    public static final double kDVelocityController = 0;
    public static final double frontKVelocityFF = 0.0021;
    public static final double backKVelocityFF = 0.0021;

    public static final Angle kMotorReduction = Rotations.of(1);
    public static final AngularVelocity kVelocityConversionFactor = kMotorReduction.div(Seconds.of(60));

    // Velocity setpoints
    // public static final AngularVelocity kVelocityFront =
    // RadiansPerSecond.of(220);
    // public static final AngularVelocity kVelocityBack = RadiansPerSecond.of(275);
    public static final AngularVelocity kVelocityFront = RadiansPerSecond.of(202); // 205
    public static final AngularVelocity kVelocityBack = RadiansPerSecond.of(257); // 260
    public static final AngularVelocity kVelocityYeetBack = RadiansPerSecond.of(520);
    public static final AngularVelocity kVelocityYeetForward = RadiansPerSecond.of(520);
    public static final AngularVelocity kVelocityFrontTolerance = RPM.of(247.8);
    public static final AngularVelocity kVelocityBackTolerance = RPM.of(247.8);
  }

  public static class FieldConstants {
    public static final AprilTagFieldLayout kfieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final Distance kFieldLengthX = Meters.of(kfieldLayout.getFieldLength()); // meters from drivestation
                                                                                           // wall to
    // drivestation wall
    public static final Distance kFieldWidthY = Meters.of(kfieldLayout.getFieldWidth()); // meters of parallel distance
                                                                                         // from
    // processor to processor
    public static final Distance kBargeX = kFieldLengthX.div(2); // meters from drivestation wall to middle of barge
  }

  // PDP CAN IDs
  public static final int kPDPCanId = 0;
}
