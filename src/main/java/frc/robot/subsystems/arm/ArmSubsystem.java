// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.ColorHelpers;

/**
 * The ArmSubsystem class represents the robot's arm subsystem. It uses PIDF to
 * pivot the arm.
 */
public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armSpark;
  private final RelativeEncoder armEncoder;
  private final SparkClosedLoopController armClosedLoopController;
  private boolean usePid = true;
  private double angularOffsetDeg = 0;
  private double pidReferencePositionDegWithoutOffset;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(ArmConstants.kColorSensorI2CPort);
  private boolean algaeOnArm = false;
  private Color detectedColor;
  private int proximity;

  /**
   * The constructor for the ArmSubsystem class configures the arm motor.
   */
  public ArmSubsystem(
      SparkMax armSpark,
      int limitSwitchChannel) {
    this.armSpark = armSpark;

    armEncoder = armSpark.getEncoder();
    armClosedLoopController = armSpark.getClosedLoopController();

    resetPosition();
    stopArm();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Arm/Measured/Position", new Rotation2d(getPosition()));

    // Convert radians per second to RPM
    Logger.recordOutput("Arm/Measured/Velocity", getVelocity() / ArmConstants.kVelocityConversionFactor);

    Logger.recordOutput("Arm/Setpoint/AngularOffset", Rotation2d.fromDegrees(angularOffsetDeg));

    Logger.recordOutput("Arm/UsePid", usePid);

    Logger.recordOutput("Arm/Measured/ArmPose", new Pose3d(0.3175, 0, 0.717551,
        new Rotation3d(0, RobotBase.isReal() ? getPosition() : getReferencePosition(), 0)));

    if (usePid) {
      double setpoint = getReferencePosition();

      Logger.recordOutput("Arm/Setpoint/Position",
          new Rotation2d(setpoint));

      double gravityCompensationFFVoltage = ArmConstants.kGainFF * Math.sin(getPosition());
      REVLibError isOk = armClosedLoopController.setReference(
          setpoint,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          gravityCompensationFFVoltage);
      Logger.recordOutput("Arm/Setpoint/PositionOk", isOk);
    }

    // Check whether algae is on arm
    detectAlgae();
    Logger.recordOutput("Arm/ColorSensor/AlgaeDetected", algaeOnArm);
  }

  /**
   * Returns the current arm position.
   * 
   * @return double The current angle of the arm, in radians.
   */
  public double getPosition() {
    // Adds the angular offset
    return armEncoder.getPosition();
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
   * <strong>This method will ONLY start working after given a non zero
   * input, which then manual mode will be enabled.</strong> After enabling
   * manual mode, an input of zero will command the arm to run at 0% velocity
   * (stopping it).
   * See {@link #setVelocity(double, boolean)} to force manual mode even if the
   * input is zero.
   * Use {@link #setReferencePosition(double)} to re-enable PID control.
   * 
   * @param percent Percent output, from -1 to 1.
   */
  public void setVelocity(double percent) {
    if (percent != 0) {
      usePid = false;
    }
    if (usePid)
      return;

    Logger.recordOutput("Arm/Setpoint/PercentVelocity", percent);
    armSpark.set(percent);
  }

  /**
   * Sets the velocity of the arm.
   * 
   * This method allows the enforcement of manual mode, which will then command
   * the arm to run at percent velocities.
   * See {@link #setVelocity(double)} to enable manual mode automatically (when
   * the input is non zero)
   * Use {@link #setReferencePosition(double)} to re-enable PID control.
   * 
   * @param percent     Percent output, from -1 to 1.
   * @param forceManual Whether to force manual control and stop Pid
   */
  public void setVelocity(double percent, boolean forceManual) {
    if (forceManual) {
      usePid = false;
    }
    setVelocity(percent);
  }

  /**
   * Sets the reference position for the arm closed loop controller.
   * 
   * <strong>This method will disable manual mode and enable PID control.</strong>
   * 
   * @param referenceAngle The reference angle, in degrees.
   */
  public void setReferencePosition(double referenceAngleDeg) {
    usePid = true;

    pidReferencePositionDegWithoutOffset = referenceAngleDeg;
    // double offsettedReferenceAngleDeg = referenceAngleDeg + angularOffsetDeg;

    // // Convert deg -> rad
    // double refAngleRad = offsettedReferenceAngleDeg *
    // ArmConstants.kAngleConversionFactor;

    // double gravityCompensationFFVoltage = ArmConstants.kGainFF *
    // Math.sin(getPosition());

    // Logger.recordOutput("Arm/Setpoint/Position",
    // new Rotation2d(refAngleRad));
    // REVLibError isOk = armClosedLoopController.setReference(refAngleRad,
    // ControlType.kPosition,
    // ClosedLoopSlot.kSlot0, gravityCompensationFFVoltage);
    // Logger.recordOutput("Arm/Setpoint/PositionOk", isOk);
  }

  /**
   * Returns the setpoint, including offset for the arm closed loop controller.
   * 
   * @return double The reference angle, in radians.
   */
  public double getReferencePosition() {
    double offsettedReferenceAngleDeg = pidReferencePositionDegWithoutOffset + angularOffsetDeg;

    // Convert deg -> rad
    return offsettedReferenceAngleDeg * ArmConstants.kAngleConversionFactor;
  }

  /**
   * Stops the arm motor using percent output.
   */
  public void stopArm() {
    setVelocity(0, true);
  }

  /**
   * Resets the arm encoder to 0.
   */
  public void resetPosition() {
    armEncoder.setPosition(0);
  }

  /**
   * Calibrates the arm encoder to the starting angle.
   */
  public void calibrate() {
    armEncoder.setPosition(ArmConstants.kAngleStart);
  }

  /**
   * Returns whether the arm is using PID.
   * 
   * @return boolean Whether the arm is using PID.
   */
  public boolean getIsUsingPid() {
    return usePid;
  }

  /**
   * Returns the angular offset of the arm.
   * 
   * @return The angular offset of the arm, in degrees.
   */
  public double getAngularOffset() {
    return angularOffsetDeg;
  }

  /**
   * Resets the angular offset of the arm to 0 degrees.
   */
  public void clearOffset() {
    setAngularOffset(0);
  }

  /**
   * Sets the angular offset of the arm.
   * 
   * @param offsetDeg The angular offset of the arm, in degrees.
   */
  public void setAngularOffset(double offsetDeg) {
    angularOffsetDeg = offsetDeg;
  }

  /**
   * Increases the angular offset of the arm. Use a negative value to decrease the
   * offset.
   * 
   * @param offsetDeg The angular offset of the arm, in degrees. Can be negative.
   */
  public void increaseAngularOffset(double offsetDeg) {
    angularOffsetDeg += offsetDeg;
  }

  /**
   * Returns whether algae is detected to be currently on the arm.
   * 
   * @return algaeOnArm boolean variable
   */
  public boolean getAlgaeOnArm() {
    return algaeOnArm;
  }

  /**
   * Sets the value for whether algae is detected to be currently on the arm.
   * 
   * @param algaeDetected boolean value
   */
  public void setAlgaeOnArm(boolean algaeDetected) {
    algaeOnArm = algaeDetected;
  }

  /**
   * Called periodically to determine whether algae is currently on the arm, using
   * the color sensor.
   */
  public void detectAlgae() {
    // Get values from color sensor
    detectedColor = colorSensor.getColor();

    // The IR sensor detects the object's proximity
    // Returns a value from 0 to 2047, with 2047 being the closest
    proximity = colorSensor.getProximity();

    Logger.recordOutput("Arm/ColorSensor/DetectedColorHex", detectedColor.toHexString());
    Logger.recordOutput("Arm/ColorSensor/DetectedColorName", detectedColor.toString());
    Logger.recordOutput("Arm/ColorSensor/Proximity", proximity);

    // Check if detected color is within the proximity threshold and whether it
    // matches the color of algae
    if (proximity < ArmConstants.kColorSensorProximityThreshold
        && ColorHelpers.colorsMatch(detectedColor, ArmConstants.kAlgaeColor, ArmConstants.kColorMatchThreshold)) {
      setAlgaeOnArm(true);
    } else {
      setAlgaeOnArm(false);
    }
  }

}
