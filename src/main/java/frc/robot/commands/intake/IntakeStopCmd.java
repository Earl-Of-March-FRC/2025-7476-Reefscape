// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * This command stops the intake rollers.
 */
public class IntakeStopCmd extends InstantCommand {

  private IntakeSubsystem intakeSub;

  /**
   * Sets the reference value for the intake subsystem's closed-loop controller.
   * It will slow down to a stop.
   * 
   * @param intakeSub The instance of the IntakeSubsystem class to be
   *                  used.
   */
  public IntakeStopCmd(IntakeSubsystem intakeSub) {
    this.intakeSub = intakeSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSub.setReferenceVelocity(0);
  }
}
