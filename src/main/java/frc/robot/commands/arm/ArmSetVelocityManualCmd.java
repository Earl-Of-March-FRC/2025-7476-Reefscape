// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

/**
 * This command moves the arm manually.
 */
public class ArmSetVelocityManualCmd extends Command {

  private ArmSubsystem armSub;
  private DoubleSupplier percent;

  /**
   * Moves the arm manually.
   * 
   * @param armSub  The instance of the ArmSubsystem class to be used.
   * @param percent Percent output, from -1 to 1.
   */
  public ArmSetVelocityManualCmd(ArmSubsystem armSub, DoubleSupplier percent) {
    this.armSub = armSub;
    this.percent = percent;

    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double percentDouble = percent.getAsDouble();

    // If joystick has not been moved, then don't do anything
    // Prevents this from interrupting the arm from holding its position with PID
    if (percentDouble != 0) {
      armSub.setVelocity(percentDouble);
    }
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
