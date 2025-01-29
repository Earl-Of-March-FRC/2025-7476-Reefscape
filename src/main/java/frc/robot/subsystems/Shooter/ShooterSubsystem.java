// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax topShooterSpark = new SparkMax(ShooterConstants.kTopShooterMotorPort,
      SparkMax.MotorType.kBrushless);
  private final RelativeEncoder topShooterEncoder = topShooterSpark.getEncoder();
  private final SparkClosedLoopController topShooterClosedLoopController = topShooterSpark.getClosedLoopController();

  private final SparkMax bottomShooterSpark = new SparkMax(ShooterConstants.kTopShooterMotorPort,
      SparkMax.MotorType.kBrushless);
  private final RelativeEncoder bottomShooterEncoder = bottomShooterSpark.getEncoder();
  private final SparkClosedLoopController bottomShooterClosedLoopController = bottomShooterSpark
      .getClosedLoopController();

  public ShooterSubsystem() {
    // configures shooter motors
    topShooterSpark.configure(ShooterConfigs.shooterConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    bottomShooterSpark.configure(ShooterConfigs.shooterConfig.inverted(true), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    topShooterSpark.set(speed);
    bottomShooterSpark.set(speed);
  }

  public double getShooterVelocity() {
    return topShooterEncoder.getVelocity();
  }

  // sets the reference velocity for the shooter controller
  public void setReferenceSpeed(double referenceSpeed) {
    topShooterClosedLoopController.setReference(referenceSpeed, ControlType.kVelocity);
    bottomShooterClosedLoopController.setReference(referenceSpeed, ControlType.kVelocity);
  }

}
