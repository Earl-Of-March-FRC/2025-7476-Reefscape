// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.Launcher;

/**
 * Command for controlling the launcher using a PID loop.
 * <p>
 * This command sets a target speed for the launcher and continuously monitors
 * the current speed of the launcher subsystem.
 */
public class LauncherSetVelocityPIDCmd extends Command {

  private final Launcher launcherSub;
  private final Supplier<AngularVelocity> frontReferenceVelocity;
  private final Supplier<AngularVelocity> backReferenceVelocity;

  /**
   * Constructs a new LauncherSetVelocityPID.
   * 
   * @param launcherSub            The launcher subsystem that this command will
   *                               control.
   * @param frontReferenceVelocity The target speed for the front launcher, in
   *                               RPM.
   * @param backReferenceVelocity  The target speed for the back launcher, in RPM.
   */
  public LauncherSetVelocityPIDCmd(Launcher launcherSub, Supplier<AngularVelocity> frontReferenceVelocity,
      Supplier<AngularVelocity> backReferenceVelocity) {
    this.launcherSub = launcherSub;
    this.frontReferenceVelocity = frontReferenceVelocity;
    this.backReferenceVelocity = backReferenceVelocity;

    addRequirements(launcherSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherSub.setFrontReferenceVelocity(frontReferenceVelocity.get());
    launcherSub.setBackReferenceVelocity(backReferenceVelocity.get());
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
