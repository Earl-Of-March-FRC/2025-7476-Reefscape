// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase implements IntakeInterface {
  private final SparkMax shoulder = new SparkMax(IntakeConstants.kShoulderMotorPort, SparkMax.MotorType.kBrushless);
  private final SparkMax rollers = new SparkMax(IntakeConstants.kRollersMotorPort, SparkMax.MotorType.kBrushless);
  private final SparkMaxConfig shoulderConfig = new SparkMaxConfig();
  private final SparkMaxConfig rollerConfig = new SparkMaxConfig();

  private final Encoder shoulderEncoder = new Encoder(0, 0);

  // private final RelativeEncoder shoulderEncoder = shoulder.getEncoder();
  // private final SparkClosedLoopController shoulderController =
  // shoulder.getClosedLoopController();

  private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
  private final PIDController pid = new PIDController(0, 0, 0);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // shoulderConfig.setInverted(false);
    // rollers.setInverted(false);
    shoulderConfig.idleMode(IdleMode.kBrake);
    rollerConfig.idleMode(IdleMode.kBrake);
    // pivotEncoder.setDistancePerPulse(360.00 / 2048.00); // This method does not
    // exist for RelativeEncoder

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm position", getShoulderPosition());
    SmartDashboard.putBoolean("Arm More Than 107", getShoulderPosition() > 107);

  }

  public void setRollerSpeed(double speed) {
    rollers.set(speed);
  }

  public void setShoulderSpeed(double speed) {
    shoulder.set(speed);
  }

  public void shoulderHold(double currentAngle) {
    final double feed = feedforward.calculate(Math.toRadians(currentAngle), 0);

    final double output = pid.calculate(getShoulderRate(), 0);

    shoulder.setVoltage(output + feed);

  }

  public double getShoulderPosition() {
    return shoulderEncoder.getDistance();
  }

  public double getShoulderRate() {
    return shoulderEncoder.getRate();
  }
}
