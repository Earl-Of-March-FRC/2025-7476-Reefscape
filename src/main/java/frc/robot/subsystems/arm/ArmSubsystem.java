// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
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

  // Starting angle of the arm, in radians
  // Ex: arm starting position is 1 radian, then m_armAngularOffset is 1
  private double m_armAngularOffset = 0;

  /**
   * The constructor for the ArmSubsystem class configures the arm motor.
   */
  public ArmSubsystem() {

    // Configures arm motor
    armSpark.configure(ArmConfigs.armConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    resetPosition();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Arm/Measured/Position", new Rotation2d(getPosition()));

    // Convert radians per second to RPM
    Logger.recordOutput("Intake/Arm/Measured/Velocity", getVelocity() / ArmConstants.kVelocityConversionFactor);
  }

  /**
   * Returns the current arm position.
   * 
   * @return double The current angle of the arm, in radians.
   */
  public double getPosition() {
    // Adds the angular offset
    return armEncoder.getPosition() + m_armAngularOffset;
  }

  /**
   * Returns the current arm velocity.
   * 
   * @return double The current velocity of the arm, in radians per second.
   */
  public double getVelocity() {
    return armEncoder.getVelocity();
  }

  /**
   * Sets the velocity of the arm.
   * 
   * @param velocity Percent output, from -1 to 1.
   */
  public void setVelocity(double velocity) {
    Logger.recordOutput("Intake/Arm/Setpoint/PercentVelocity", velocity);
    armSpark.set(velocity);
  }

  /**
   * Sets the reference position for the arm closed loop controller.
   * 
   * @param referenceAngle The reference angle, in degrees.
   */
  public void setReferencePosition(double referenceAngle) {
    // Convert to radians, then subtract angular offset
    double refAngleWithOffset = referenceAngle * ArmConstants.kAngleConversionFactor - m_armAngularOffset;

    Logger.recordOutput("Intake/Arm/Setpoint/Position",
        new Rotation2d(referenceAngle * ArmConstants.kAngleConversionFactor));
    armClosedLoopController.setReference(refAngleWithOffset, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /**
   * Sets the reference velocity for the arm closed loop controller.
   *
   * @param referenceVelocity The reference velocity, in RPM.
   */
  public void setReferenceVelocity(double referenceVelocity) {
    // Convert RPM to radians per second
    double refVelocityConverted = referenceVelocity * ArmConstants.kVelocityConversionFactor;

    Logger.recordOutput("Intake/Arm/Setpoint/Velocity", referenceVelocity);
    armClosedLoopController.setReference(refVelocityConverted,
        ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  /**
   * Stops the arm motor.
   */
  public void stopArm() {
    setVelocity(0);
  }

  /**
   * Resets the arm encoder to 0.
   */
  public void resetPosition() {
    armEncoder.setPosition(0);
  }
}
