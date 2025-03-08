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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ArmConfigs;
import frc.robot.Constants.ArmConstants;

/**
 * The ArmSubsystem class represents the robot's arm subsystem. It uses PIDF to
 * pivot the arm.
 */
public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armSpark;
  private final RelativeEncoder armEncoder;
  private final SparkClosedLoopController armClosedLoopController;
  private final DigitalInput lowerLimitSwitch;
  public boolean isManual = false;
  public double armOffset = 0;

  // Starting angle of the arm, in radians
  // Ex: arm starting position is 1 radian, then m_armAngularOffset is 1
  private double m_armAngularOffset = 0;

  /**
   * The constructor for the ArmSubsystem class configures the arm motor.
   */
  public ArmSubsystem(SparkMax armSpark, int limitSwitchChannel) {
    this.armSpark = armSpark;

    lowerLimitSwitch = new DigitalInput(limitSwitchChannel);

    armEncoder = armSpark.getEncoder();
    armClosedLoopController = armSpark.getClosedLoopController();

    // Configures arm motor
    armSpark.configure(ArmConfigs.armConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    resetPosition();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Arm/Measured/Position", new Rotation2d(getPosition()));

    // Convert radians per second to RPM
    Logger.recordOutput("Arm/Measured/Velocity", getVelocity() / ArmConstants.kVelocityConversionFactor);
    Logger.recordOutput("Arm/Measured/armOffset", this.armOffset);

    Logger.recordOutput("Arm/Measured/limitSwitch", getLimitSwitch());
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
   * @param percent Percent output, from -1 to 1.
   */
  public void setVelocity(double percent) {
    Logger.recordOutput("Arm/Setpoint/PercentVelocity", percent);
    armSpark.set(percent);
  }

  /**
   * Sets the reference position for the arm closed loop controller.
   * 
   * @param referenceAngle The reference angle, in degrees.
   */
  public void setReferencePosition(double referenceAngle) {
    // Convert to radians, then subtract angular offset
    double refAngleWithOffset = referenceAngle * ArmConstants.kAngleConversionFactor - m_armAngularOffset;

    // Determine whether arm needs to move up or down
    ClosedLoopSlot closedLoopSlot;

    // If arm needs to move up to reach reference position, use the upward
    // closed-loop controller
    // Note: arm moving upward is in the negative direction
    if (getPosition() >= refAngleWithOffset) {
      closedLoopSlot = ClosedLoopSlot.kSlot0;
    }

    // Otherwise, use the downward closed-loop controller
    else {
      closedLoopSlot = ClosedLoopSlot.kSlot1;
    }

    Logger.recordOutput("Arm/Setpoint/Position",
        new Rotation2d(referenceAngle * ArmConstants.kAngleConversionFactor));
    armClosedLoopController.setReference(refAngleWithOffset, ControlType.kPosition, closedLoopSlot);
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

  /**
   * Check if the calibration limit switches are triggered.
   * 
   * @return {@code true} if the limit switch is pressed, {@code false} if not.
   */
  public boolean getLimitSwitch() {
    return !lowerLimitSwitch.get();
  }
}
