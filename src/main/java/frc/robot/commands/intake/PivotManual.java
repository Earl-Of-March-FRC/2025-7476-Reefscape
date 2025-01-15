// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotManual extends Command {
  /** Creates a new PivotManual. */

  private IntakeSubsystem intakeSub;
  private DoubleSupplier speed;

  SlewRateLimiter filter = new SlewRateLimiter(0.9);

  public PivotManual(IntakeSubsystem intakeSub, DoubleSupplier d) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.intakeSub = intakeSub;
    this.speed = d;

    addRequirements(intakeSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(speed.getAsDouble()) < 0.3) {
      intakeSub.setPivotSpeed(speed.getAsDouble());
    } else {
      intakeSub.setPivotSpeed(filter.calculate(speed.getAsDouble()) * 0.7);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
