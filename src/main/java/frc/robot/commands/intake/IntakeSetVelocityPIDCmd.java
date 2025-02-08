// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * This command uses closed-loop control to set the intake rollers to the
 * desired velocity.
 */
public class IntakeSetVelocityPIDCmd extends InstantCommand {

  private IntakeSubsystem intakeSub;
  private double referenceVelocity;

  /**
   * Sets the reference value for the intake subsystem's closed-loop controller.
   * It will automatically move towards the reference velocity.
   * 
   * @param intakeSub         The instance of the IntakeSubsystem class to be
   *                          used.
   * @param referenceVelocity The reference velocity for the intake rollers to
   *                          move to, in RPM.
   */
  public IntakeSetVelocityPIDCmd(IntakeSubsystem intakeSub, double referenceVelocity) {
    this.intakeSub = intakeSub;
    this.referenceVelocity = referenceVelocity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSub.setReferenceVelocity(referenceVelocity);
  }
}
