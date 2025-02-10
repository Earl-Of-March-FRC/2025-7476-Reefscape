// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.launcher.LauncherSubsystem;

/**
 * This command stops the launcher using percent output.
 */
public class LauncherStopCmd extends InstantCommand {

  private LauncherSubsystem launcherSub;

  /**
   * Constructs a new LauncherStopCmd.
   * 
   * @param launcherSub The instance of the LauncherSubsystem class to be
   *                    used.
   */
  public LauncherStopCmd(LauncherSubsystem launcherSub) {
    this.launcherSub = launcherSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcherSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherSub.setVelocity(0);
  }
}
