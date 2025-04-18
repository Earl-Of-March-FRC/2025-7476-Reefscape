package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class GyroADXRS450 implements Gyro {
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public GyroADXRS450() {

  }

  /**
   * {@inheritDoc}
   */
  @Override
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public double getRate() {
    return gyro.getRate();
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void calibrate() {
    gyro.calibrate();
    gyro.reset();
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public boolean isConnected() {
    return gyro.isConnected();
  }

}