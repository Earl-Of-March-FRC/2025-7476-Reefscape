// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveCmd extends Command {

  private Drivetrain driveSub;
  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;
  private Supplier<Double> omegaSupplier;

  public DriveCmd(Drivetrain driveSub, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier) {
    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;

    addRequirements(driveSub);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double xVel = xSupplier.get() * DriveConstants.kMaxSpeedMetersPerSecond;
    double yVel = ySupplier.get() * DriveConstants.kMaxSpeedMetersPerSecond;
    double omega = omegaSupplier.get() * DriveConstants.kMaxAngularSpeed;
    driveSub.runVelocityFieldRelative(new ChassisSpeeds(xVel, yVel, omega));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
