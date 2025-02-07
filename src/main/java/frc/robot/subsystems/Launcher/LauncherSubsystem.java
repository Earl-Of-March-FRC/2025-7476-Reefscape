package frc.robot.subsystems.Launcher;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.LauncherConfigs;

/**
 * LauncherSubsystem controls the shooter mechanism of the robot.
 * It manages motor speed, velocity, and PID control for the shooter.
 */
public class LauncherSubsystem extends SubsystemBase {
    private final SparkMax topLauncherSpark = new SparkMax(2, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder topLauncherEncoder = topLauncherSpark.getEncoder();
    private final SparkClosedLoopController topLauncherClosedLoopController = topLauncherSpark
            .getClosedLoopController();

    private final SparkMax bottomLauncherSpark = new SparkMax(1, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder bottomLauncherEncoder = bottomLauncherSpark.getEncoder();
    private final SparkClosedLoopController bottomLauncherClosedLoopController = bottomLauncherSpark
            .getClosedLoopController();

    /**
     * Constructs a new LauncherSubsystem and configures the shooter motors.
     */
    public LauncherSubsystem() {
        // Configure motors
        topLauncherSpark.configure(LauncherConfigs.shooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        bottomLauncherSpark.configure(LauncherConfigs.shooterConfig.inverted(true), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /**
     * This method is called periodically by the scheduler.
     * It publishes shooter velocity to the SmartDashboard.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("TopLauncherVelocity", topLauncherEncoder.getVelocity());
        SmartDashboard.putNumber("BottomLauncherVelocity", bottomLauncherEncoder.getVelocity());
    }

    /**
     * Sets the speed of both shooter motors.
     *
     * @param speed The speed to set the motors to, typically in the range [-1.0,
     *              1.0].
     */
    public void setLauncherSpeed(double speed) {
        topLauncherSpark.set(speed);
        bottomLauncherSpark.set(speed);
    }

    /**
     * Gets the velocity of the top shooter motor.
     *
     * @return The velocity of the top shooter motor in RPM.
     */
    public double getLauncherVelocity() {
        return topLauncherEncoder.getVelocity();
    }

    /**
     * Sets a reference speed for the shooter motors using closed-loop velocity
     * control.
     *
     * @param referenceSpeed The target speed in RPM.
     */
    public void setReferenceSpeed(double referenceSpeed) {
        topLauncherClosedLoopController.setReference(referenceSpeed, ControlType.kVelocity);
        bottomLauncherClosedLoopController.setReference(referenceSpeed, ControlType.kVelocity);
    }
}
