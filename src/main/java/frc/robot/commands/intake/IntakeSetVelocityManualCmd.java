// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * This command sets the speed of the intake rollers using percent output.
 */
public class IntakeSetVelocityManualCmd extends Command {

  private IntakeSubsystem intakeSub;
  private double speed;

  /**
   * Sets the speed of the intake rollers.
   * 
   * @param intakeSub The instance of the IntakeSubsystem class to be used.
   * @param speed     Desired speed, from -1 to 1.
   */
  public IntakeSetVelocityManualCmd(IntakeSubsystem intakeSub, double speed) {

    this.intakeSub = intakeSub;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.setIntakeVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
