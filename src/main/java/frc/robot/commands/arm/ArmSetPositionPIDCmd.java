// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

/**
 * This command uses closed-loop control to move the arm to the desired angle.
 */
public class ArmSetPositionPIDCmd extends Command {

  private ArmSubsystem armSub;
  private double referenceAngle;

  /**
   * Sets the reference value for the arm subsystem's closed-loop controller. It
   * will automatically begin moving towards the desired angle.
   * 
   * @param armSub         The instance of the ArmSubsystem class to be used.
   * @param referenceAngle The goal angle for the arm to move to, in degrees.
   */
  public ArmSetPositionPIDCmd(ArmSubsystem armSub, double referenceAngle) {

    this.armSub = armSub;
    this.referenceAngle = referenceAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSub.setReferencePosition(referenceAngle);
    armSub.isManual = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // If the command has been interrupted before the arm has reached its setpoint,
    // then set the current arm angle as the new reference angle using position PID
    // If it has already reached the setpoint, it should continue holding its
    // current position
    if (Math.abs(
        referenceAngle - armSub.getPosition() / ArmConstants.kAngleConversionFactor) > ArmConstants.kAngleTolerance) {

      // Convert the current position to degrees
      armSub.setReferencePosition(armSub.getPosition() /
          ArmConstants.kAngleConversionFactor);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSub.isManual;
  }
}
