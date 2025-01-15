// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase implements IntakeInterface {

  private final WPI_TalonSRX pivot = new WPI_TalonSRX(IntakeConstants.pivotMotorPort);
  private final WPI_TalonSRX rollers = new WPI_TalonSRX(IntakeConstants.rollersMotorPort);

  private final Encoder pivotEncoder = new Encoder(0, 0);

  private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
  private final PIDController pid = new PIDController(0, 0, 0);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    pivot.setInverted(false);
    rollers.setInverted(false);

    pivot.setNeutralMode(NeutralMode.Brake);
    rollers.setNeutralMode(NeutralMode.Brake);

    pivotEncoder.setDistancePerPulse(360.00 / 2048.00);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm position", getPivotPosition());
    SmartDashboard.putBoolean("Arm More Than 107", getPivotPosition() > 107);

  }

  public void setRollerSpeed(double speed) {
    rollers.set(speed);
  }

  public void setPivotSpeed(double speed) {
    pivot.set(speed);
  }

  public void pivotHold(double currentAngle) {
    final double feed = feedforward.calculate(Math.toRadians(currentAngle), 0);

    final double output = pid.calculate(getPivotRate(), 0);

    pivot.setVoltage(output + feed);

  }

  public double getPivotPosition() {
    return pivotEncoder.getDistance();
  }

  public double getPivotRate() {
    return pivotEncoder.getRate();
  }

}
