package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
    public Rotation2d getRotation2d();

    public double getRate();

    public default void calibrate() {
    }
}