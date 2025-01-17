// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterAutoCmd extends Command {
  private ShooterSubsystem shooterSub;
  private double motorSpeed, timeout;
  private Timer timer = new Timer();

  /** Creates a new ShooterAutoCmd. */
  public ShooterAutoCmd(ShooterSubsystem shooter, double speed, double timeout) {
    this.shooterSub = shooter;
    motorSpeed = speed;
    this.timeout = timeout;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() >= timeout) {
      shooterSub.shoot(0);
    } else if (timer.get() < timeout / 2) {
      shooterSub.shoot(motorSpeed);
    } else {
      shooterSub.shoot(-motorSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    shooterSub.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= timeout);
  }
}