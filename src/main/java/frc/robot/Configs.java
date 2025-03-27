package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      Distance drivingFactor = ModuleConstants.kWheelCircumference.div(ModuleConstants.kDrivingMotorReduction);
      Angle turningFactor = Radians.of(2 * Math.PI);
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeed.in(RotationsPerSecond);

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor.in(Meters)) // meters
          .velocityConversionFactor(drivingFactor.div(60.0).in(Meters)); // meters per second
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20);
      turningConfig.absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor.in(Radians)) // radians
          .velocityConversionFactor(turningFactor.div(60.0).in(Radians)); // radians per second
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor.in(Radians));
    }
  }

  public static final class ArmConfigs {
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();

    static {

      armConfig
          .idleMode(IdleMode.kBrake) // Set to kBrake to hold position when not moving
          .smartCurrentLimit(40); // Adjust current limit as needed

      armConfig.encoder
          .positionConversionFactor(ArmConstants.kPositionConversionFactor.in(Radians)) // Radians
          .velocityConversionFactor(ArmConstants.kVelocityConversionFactor.in(RadiansPerSecond)); // Radians per second

      armConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Upward position PID controller is slot 0
          .pidf(ArmConstants.kPPositionController, ArmConstants.kIPositionController,
              ArmConstants.kDPositionController, ArmConstants.kPositionFF, ClosedLoopSlot.kSlot0)

          // Downward position PID controller is slot 1
          // .pidf(ArmConstants.kPDownPositionController,
          // ArmConstants.kIDownPositionController,
          // ArmConstants.kDDownPositionController, ArmConstants.kDownPositionFF,
          // ClosedLoopSlot.kSlot1)
          .outputRange(-1, 1);
    }
  }

  public static final class IntakeConfigs {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      intakeConfig
          .idleMode(IdleMode.kCoast) // Set to kBrake to hold position when not moving
          .smartCurrentLimit(30); // Adjust current limit as needed

      intakeConfig.encoder
          .positionConversionFactor(IntakeConstants.kPositionConversionFactor.in(Radians))
          .velocityConversionFactor(IntakeConstants.kVelocityConversionFactor.in(RadiansPerSecond));
    }
  }

  public static final class IndexerConfigs {

    public static final SparkMaxConfig indexerConfig = new SparkMaxConfig();

    static {
      indexerConfig.idleMode(IdleMode.kBrake);
      indexerConfig.smartCurrentLimit(40);
      indexerConfig.encoder
          .velocityConversionFactor(IndexerConstants.kWheelDiameterMeters * Math.PI
              / IndexerConstants.kMotorReduction / 60);
    }
  }

  public static final class LauncherConfigs {
    public static final SparkMaxConfig frontLauncherConfig = new SparkMaxConfig();
    public static final SparkMaxConfig backLauncherConfig = new SparkMaxConfig();

    static {
      frontLauncherConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50)
          .voltageCompensation(10);
      frontLauncherConfig.encoder
          .positionConversionFactor(1)
          .velocityConversionFactor(LauncherConstants.kVelocityConversionFactor.in(RadiansPerSecond));
      frontLauncherConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(LauncherConstants.kPVelocityController,
              LauncherConstants.kIVelocityController,
              LauncherConstants.kDVelocityController,
              LauncherConstants.frontKVelocityFF)
          .outputRange(-1, 1);
    }

    static {
      backLauncherConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50)
          .voltageCompensation(10);
      backLauncherConfig.encoder
          .positionConversionFactor(1)
          .velocityConversionFactor(LauncherConstants.kVelocityConversionFactor.in(RadiansPerSecond));
      backLauncherConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(LauncherConstants.kPVelocityController,
              LauncherConstants.kIVelocityController,
              LauncherConstants.kDVelocityController,
              LauncherConstants.backKVelocityFF)
          .outputRange(-1, 1);
    }
  }
}