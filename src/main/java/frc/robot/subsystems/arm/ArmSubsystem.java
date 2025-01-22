// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ArmConfigs;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armSpark = new SparkMax(ArmConstants.kArmMotorPort, SparkMax.MotorType.kBrushless);
  private final SparkAbsoluteEncoder armEncoder = armSpark.getAbsoluteEncoder();
  private final SparkClosedLoopController armClosedLoopController = armSpark.getClosedLoopController();

  private double m_armAngularOffset = 0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    // configures arm motor
    armSpark.configure(ArmConfigs.armConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  }

  // sets the arm speed
  public void setArmSpeed(double speed) {
    armSpark.set(speed);
  }

  // returns the arm's current position
  public double getArmPosition() {
    return armEncoder.getPosition() - m_armAngularOffset;
  }

  // returns the arm's current velocity
  public double getArmVelocity() {
    return armEncoder.getVelocity();
  }

  // sets the reference value / setpoint for the arm controller
  public void setReferenceAngle(double referenceAngle) {
    armClosedLoopController.setReference(referenceAngle + m_armAngularOffset, ControlType.kPosition);
  }
}
