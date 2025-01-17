// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterRotatePID extends Command {
  ShooterSubsystem ssub;
  double degrees;

  /** Creates a new ShooterRotatePID. */
  public ShooterRotatePID(ShooterSubsystem ssub, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ssub = ssub;
    this.degrees = degrees;
    addRequirements(this.ssub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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