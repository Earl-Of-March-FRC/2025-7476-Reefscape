// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShoulderHold extends Command {
  /** Creates a new IntakeHold. */

  private IntakeSubsystem intakeSub;
  private double angle;

  public ShoulderHold(IntakeSubsystem intakeSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intakeSub = intakeSub;
    angle = intakeSub.getShoulderPosition();
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.shoulderHold(angle);
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