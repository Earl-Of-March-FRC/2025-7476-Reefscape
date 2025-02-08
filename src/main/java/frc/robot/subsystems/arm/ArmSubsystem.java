// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ArmConfigs;
import frc.robot.Constants.ArmConstants;

/**
 * The ArmSubsystem class represents the robot's arm subsystem. It uses PIDF to
 * pivot the arm.
 */
public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armSpark = new SparkMax(ArmConstants.kArmMotorCanId, SparkMax.MotorType.kBrushless);
  private final RelativeEncoder armEncoder = armSpark.getEncoder();
  private final SparkClosedLoopController armClosedLoopController = armSpark.getClosedLoopController();

  private double m_armAngularOffset = 0;

  /**
   * The constructor for the ArmSubsystem class configures the arm motor.
   */
  public ArmSubsystem() {

    // configures arm motor
    armSpark.configure(ArmConfigs.armConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Arm/Measured/Position", getArmPosition());
    Logger.recordOutput("Intake/Arm/Measured/Velocity", getArmVelocity());
  }

  /**
   * Returns the current arm position.
   * 
   * @return double The current angle of the arm, in radians.
   */
  public double getArmPosition() {
    return (armEncoder.getPosition() - m_armAngularOffset) * ArmConstants.kArmPositionReductionFactor;
  }

  /**
   * Returns the current arm velocity.
   * 
   * @return double The current velocity of the arm, in RPM.
   */
  public double getArmVelocity() {
    return armEncoder.getVelocity();
  }

  /**
   * Sets the velocity of the arm.
   * 
   * @param velocity Desired velocity, in RPM.
   */
  public void setArmVelocity(double velocity) {
    Logger.recordOutput("Intake/Arm/Setpoint/Velocity", velocity);
    armSpark.set(velocity);
  }

  /**
   * Sets the goal angle for the arm closed loop controller.
   * 
   * @param referenceAngle The reference angle, in radians.
   */
  public void setReferenceAngle(double referenceAngle) {
    double refAngleWithOffset = (referenceAngle + m_armAngularOffset) / ArmConstants.kArmPositionReductionFactor;

    Logger.recordOutput("Intake/Arm/Setpoint/Position", refAngleWithOffset);
    armClosedLoopController.setReference(refAngleWithOffset, ControlType.kPosition);
  }
}
