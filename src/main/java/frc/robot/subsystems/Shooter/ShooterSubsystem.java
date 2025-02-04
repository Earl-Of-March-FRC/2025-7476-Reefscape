package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfigs;

/**
 * ShooterSubsystem controls the shooter mechanism of the robot.
 * It manages motor speed, velocity, and PID control for the shooter.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax topShooterSpark = new SparkMax(2, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder topShooterEncoder = topShooterSpark.getEncoder();
    private final SparkClosedLoopController topShooterClosedLoopController = topShooterSpark.getClosedLoopController();

    private final SparkMax bottomShooterSpark = new SparkMax(1, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder bottomShooterEncoder = bottomShooterSpark.getEncoder();
    private final SparkClosedLoopController bottomShooterClosedLoopController = bottomShooterSpark
            .getClosedLoopController();

    /**
     * Constructs a new ShooterSubsystem and configures the shooter motors.
     */
    public ShooterSubsystem() {
        // Configure motors
        topShooterSpark.configure(ShooterConfigs.shooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        bottomShooterSpark.configure(ShooterConfigs.shooterConfig.inverted(true), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /**
     * This method is called periodically by the scheduler.
     * It publishes shooter velocity to the SmartDashboard.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("TopShooterVelocity", topShooterEncoder.getVelocity());
        SmartDashboard.putNumber("BottomShooterVelocity", bottomShooterEncoder.getVelocity());
    }

    /**
     * Sets the speed of both shooter motors.
     *
     * @param speed The speed to set the motors to, typically in the range [-1.0,
     *              1.0].
     */
    public void setShooterSpeed(double speed) {
        topShooterSpark.set(speed);
        bottomShooterSpark.set(speed);
    }

    /**
     * Gets the velocity of the top shooter motor.
     *
     * @return The velocity of the top shooter motor in RPM.
     */
    public double getShooterVelocity() {
        return topShooterEncoder.getVelocity();
    }

    /**
     * Sets a reference speed for the shooter motors using closed-loop velocity
     * control.
     *
     * @param referenceSpeed The target speed in RPM.
     */
    public void setReferenceSpeed(double referenceSpeed) {
        topShooterClosedLoopController.setReference(referenceSpeed, ControlType.kVelocity);
        bottomShooterClosedLoopController.setReference(referenceSpeed, ControlType.kVelocity);
    }
}
