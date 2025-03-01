// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroNavX implements Gyro {
  private final AHRS gyro = new AHRS(NavXComType.kUSB1);

  public GyroNavX() {

  }

  @Override
  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  @Override
  public double getRate() {
    return gyro.getRate();
  }

  @Override
  public void calibrate() {
    gyro.zeroYaw();
  }

  @Override
  public boolean isConnected() {
    return gyro.isConnected();
  }
}