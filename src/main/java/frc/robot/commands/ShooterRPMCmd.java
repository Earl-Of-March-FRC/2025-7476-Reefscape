// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterRPMCmd extends Command {

  private ShooterSubsystem shooterSub;
  private DoubleSupplier speedBottom, speedTop;
  private BooleanSupplier start;

  /** Creates a new Shoot. */
  public ShooterRPMCmd(ShooterSubsystem shooter, DoubleSupplier speedBottom, DoubleSupplier speedTop) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSub = shooter;
    this.speedBottom = speedBottom;
    this.speedTop = speedTop;
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooterSub.setSpeedPercent(speedBottom.getAsDouble(), speedTop.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}