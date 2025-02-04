// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
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
  private final SparkMax armSpark = new SparkMax(1, SparkMax.MotorType.kBrushless);
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
    double p = armEncoder.getPosition() * 360;
    SmartDashboard.putNumber("position", p);
    // System.out.println(p);
  }

  /**
   * Sets the speed of the arm.
   * 
   * @param speed Desired speed, in RPM.
   */
  public void setArmSpeed(double speed) {
    armSpark.set(speed);
  }

  /**
   * Returns the current arm position.
   * 
   * @return double The current angle of the arm, in degrees.
   */
  public double getArmPosition() {
    return armEncoder.getPosition() - m_armAngularOffset;
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
   * Sets the goal angle for the arm closed loop controller.
   * 
   * @param referenceAngle The reference angle, in degrees.
   */
  public void setReferenceAngle(double referenceAngle) {
    armClosedLoopController.setReference(referenceAngle + m_armAngularOffset, ControlType.kPosition);
  }
}
