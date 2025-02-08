package frc.robot.subsystems.Launcher;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.LauncherConfigs;
import frc.robot.Constants.LauncherConstants;

/**
 * LauncherSubsystem controls the shooter mechanism of the robot.
 * It manages motor speed, velocity, and PID control for the shooter.
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
     * Constructs a new LauncherSubsystem and configures the shooter motors.
     */
    public LauncherSubsystem() {
        // Configure motors
        frontLauncherSpark.configure(LauncherConfigs.shooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        backLauncherSpark.configure(LauncherConfigs.shooterConfig.inverted(true), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /**
     * This method is called periodically by the scheduler.
     * It publishes shooter velocity to the SmartDashboard.
     */
    @Override
    public void periodic() {

        Logger.recordOutput("Launcher/Front/Measured/Velocity",
                getFrontVelocity() / LauncherConstants.kVelocityConversionFactor);
        Logger.recordOutput("Launcher/Back/Measured/Velocity",
                getBackVelocity() / LauncherConstants.kVelocityConversionFactor);
    }

    /**
     * Sets the speed of both shooter motors.
     *
     * @param speed The speed to set the motors to, typically in the range [-1.0,
     *              1.0].
     */
    public void setVelocity(double speed) {
        frontLauncherSpark.set(speed);
        backLauncherSpark.set(speed);
    }

    /**
     * Gets the velocity of the top shooter motor.
     *
     * @return The velocity of the top shooter motor in RPM.
     */
    public double getFrontVelocity() {
        return frontLauncherEncoder.getVelocity();
    }

    public double getBackVelocity() {
        return backLauncherEncoder.getVelocity();
    }

    public void stopLauncher() {
        setReferenceVelocity(0);
    }

    /**
     * Sets a reference speed for the shooter motors using closed-loop velocity
     * control.
     *
     * @param referenceSpeed The target speed in RPM.
     */
    public void setReferenceVelocity(double referenceVelocity) {

        frontLauncherClosedLoopController.setReference(referenceVelocity, ControlType.kVelocity);
        backLauncherClosedLoopController.setReference(referenceVelocity, ControlType.kVelocity);
    }
}
