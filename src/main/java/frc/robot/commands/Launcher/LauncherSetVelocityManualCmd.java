// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.LauncherSubsystem;

/**
 * Command to set the velocity of the launcher subsystem.
 * <p>
 * This command sets the launcher velocity based on a provided value using
 * percent output.
 */
public class LauncherSetVelocityManualCmd extends Command {

  private final LauncherSubsystem launcherSub;
  private final DoubleSupplier percent;

  /**
   * Constructs a new LauncherSetVelocityManualCmd.
   * 
   * @param launcherSub The launcher subsystem that this command will control.
   * @param percent     Percent output, from -1 to 1.
   */
  public LauncherSetVelocityManualCmd(LauncherSubsystem launcherSub, DoubleSupplier percent) {
    this.launcherSub = launcherSub;
    this.percent = percent;

    addRequirements(launcherSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherSub.setVelocity(percent.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSub.stopLauncher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
