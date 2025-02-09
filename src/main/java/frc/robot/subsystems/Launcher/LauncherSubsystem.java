package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.LauncherConfigs;
import frc.robot.Constants.LauncherConstants;

/**
 * LauncherSubsystem controls the launcher mechanism of the robot.
 * It manages motor speed, velocity, and PID control for the launcher.
 */
public class LauncherSubsystem extends SubsystemBase {
  private final SparkMax frontLauncherSpark = new SparkMax(LauncherConstants.kFrontLauncherCanId,
      SparkMax.MotorType.kBrushless);
  private final RelativeEncoder frontLauncherEncoder = frontLauncherSpark.getEncoder();
  private final SparkClosedLoopController frontLauncherClosedLoopController = frontLauncherSpark
      .getClosedLoopController();

  private final SparkMax backLauncherSpark = new SparkMax(LauncherConstants.kBackLauncherCanId,
      SparkMax.MotorType.kBrushless);
  private final RelativeEncoder backLauncherEncoder = backLauncherSpark.getEncoder();
  private final SparkClosedLoopController backLauncherClosedLoopController = backLauncherSpark
      .getClosedLoopController();

  /**
   * Constructs a new LauncherSubsystem and configures the launcher motors.
   */
  public LauncherSubsystem() {
    // Configure motors
    frontLauncherSpark.configure(LauncherConfigs.launcherConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    backLauncherSpark.configure(LauncherConfigs.launcherConfig.inverted(true), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * This method is called periodically by the scheduler.
   * It logs the launcher velocity in RPM.
   */
  @Override
  public void periodic() {
    // Converts radians per second to RPM
    Logger.recordOutput("Launcher/Front/Measured/Velocity",
        getFrontVelocity() / LauncherConstants.kVelocityConversionFactor);
    Logger.recordOutput("Launcher/Back/Measured/Velocity",
        getBackVelocity() / LauncherConstants.kVelocityConversionFactor);
  }

  /**
   * Gets the velocity of the front launcher motor.
   *
   * @return The velocity of the front launcher motor, in radians per second.
   */
  public double getFrontVelocity() {
    return frontLauncherEncoder.getVelocity();
  }

  /**
   * Gets the velocity of the back launcher motor.
   *
   * @return The velocity of the back launcher motor, in radians per second.
   */
  public double getBackVelocity() {
    return backLauncherEncoder.getVelocity();
  }

  /**
   * Sets the velocity of both launcher motors using percent output.
   *
   * @param percent The percent output, from -1 to 1.
   */
  public void setVelocity(double percent) {
    Logger.recordOutput("Launcher/Front/Setpoint/PercentVelocity", percent);
    Logger.recordOutput("Launcher/Back/Setpoint/PercentVelocity", percent);

    frontLauncherSpark.set(percent);
    backLauncherSpark.set(percent);
  }

  /**
   * Sets the reference velocity for the launcher closed loop controller.
   *
   * @param referenceVelocity The reference velocity, in RPM.
   */
  public void setReferenceVelocity(double referenceVelocity) {
    Logger.recordOutput("Launcher/Front/Setpoint/Velocity", referenceVelocity);
    Logger.recordOutput("Launcher/Back/Setpoint/Velocity", referenceVelocity);

    // Converts RPM to radians per second
    frontLauncherClosedLoopController.setReference(referenceVelocity * LauncherConstants.kVelocityConversionFactor,
        ControlType.kVelocity);
    backLauncherClosedLoopController.setReference(referenceVelocity * LauncherConstants.kVelocityConversionFactor,
        ControlType.kVelocity);
  }

  /**
   * Stops the launcher motors.
   */
  public void stopLauncher() {
    setReferenceVelocity(0);
  }
}
