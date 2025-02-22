// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

/**
 * This command moves the arm using velocity PID.
 */
public class ArmSetVelocityPIDCmd extends Command {

  private ArmSubsystem armSub;
  private DoubleSupplier input;

  /**
   * Moves the arm using velocity PID.
   * 
   * @param armSub The instance of the ArmSubsystem class to be used.
   * @param input  The desired velocity of the arm, between -1 and 1.
   */
  public ArmSetVelocityPIDCmd(ArmSubsystem armSub, DoubleSupplier input) {
    this.armSub = armSub;
    this.input = input;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Multiplies joystick input by maximum arm velocity
    armSub.setReferenceVelocity(input.getAsDouble() * ArmConstants.kMaxVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
