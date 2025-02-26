// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * This command stops the intake rollers using percent output.
 */
public class IntakeStopCmd extends InstantCommand {

  private IntakeSubsystem intakeSub;

  /**
   * Constructs a new IntakeStopCmd.
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
    intakeSub.setVelocity(0);
  }
}
