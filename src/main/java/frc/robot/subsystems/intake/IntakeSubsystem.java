// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.IntakeConstants;

/**
 * The IntakeSubsystem class represents the robot's intake subsystem. It manages
 * the intake rollers.
 */
public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax intake = new SparkMax(IntakeConstants.kIntakeMotorPort, SparkMax.MotorType.kBrushless);

  /**
   * The constructor for the IntakeSubsystem configures the intake motor.
   */
  public IntakeSubsystem() {
    intake.configure(IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the intake motor.
   * 
   * @param speed Desired speed, from -1 to 1.
   */
  public void setIntakeSpeed(double speed) {
    intake.set(speed);
  }
}
