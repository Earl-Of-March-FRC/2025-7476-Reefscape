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

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax topShooterSpark = new SparkMax(2, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder topShooterEncoder = topShooterSpark.getEncoder();
    private final SparkClosedLoopController topShooterClosedLoopController = topShooterSpark.getClosedLoopController();

    private final SparkMax bottomShooterSpark = new SparkMax(1, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder bottomShooterEncoder = bottomShooterSpark.getEncoder();
    private final SparkClosedLoopController bottomShooterClosedLoopController = bottomShooterSpark
            .getClosedLoopController();

    public ShooterSubsystem() {
        // Configure motors
        topShooterSpark.configure(ShooterConfigs.shooterConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        bottomShooterSpark.configure(ShooterConfigs.shooterConfig.inverted(true), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // Publish shooter velocity to SmartDashboard
        SmartDashboard.putNumber("TopShooterVelocity", topShooterEncoder.getVelocity());
        SmartDashboard.putNumber("BottomShooterVelocity", bottomShooterEncoder.getVelocity());
    }

    public void setShooterSpeed(double speed) {
        topShooterSpark.set(speed);
        bottomShooterSpark.set(speed);
    }

    public double getShooterVelocity() {
        return topShooterEncoder.getVelocity();
    }

    public void setReferenceSpeed(double referenceSpeed) {
        topShooterClosedLoopController.setReference(referenceSpeed, ControlType.kVelocity);
        bottomShooterClosedLoopController.setReference(referenceSpeed, ControlType.kVelocity);
    }
}