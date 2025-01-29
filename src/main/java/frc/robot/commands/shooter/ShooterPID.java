// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterPID extends InstantCommand {

  private ShooterSubsystem shooterSub;
  private DoubleSupplier goalSpeed;

  public ShooterPID(ShooterSubsystem shooterSub, DoubleSupplier goalSpeed) {
    this.shooterSub = shooterSub;
    this.goalSpeed = goalSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSub.setReferenceSpeed(goalSpeed.getAsDouble());
  }
}
