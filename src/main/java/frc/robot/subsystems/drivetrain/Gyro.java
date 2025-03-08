package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
  public Rotation2d getRotation2d();

  /*
   * Gets the rotational velocity in degrees per second
   */
  public double getRate();

  public default void calibrate() {
  }

  public default void setAngle(Rotation2d angle) {
  }

  public boolean isConnected();
}