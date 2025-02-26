// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToAlgaeCmd extends Command {
  private final AlgaeSubsystem algaeSubsystem;
  private final Trigger buttonTrigger;
  private boolean wasButtonPressed = false;

  /** Creates a new GoToAlgaeCmd. */
  public GoToAlgaeCmd(AlgaeSubsystem algaeSubsystem, Trigger btn) {
    this.algaeSubsystem = algaeSubsystem;
    this.buttonTrigger = btn;
    this.addRequirements(algaeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isButtonPressed = buttonTrigger.getAsBoolean();

    if (isButtonPressed && !wasButtonPressed) {
      // If buton is pressed track and find algae
      algaeSubsystem.updateTargetPose();
    } else if (!isButtonPressed && wasButtonPressed) {
      // stop tracking

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
