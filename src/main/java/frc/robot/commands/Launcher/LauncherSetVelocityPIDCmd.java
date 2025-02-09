package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.launcher.LauncherSubsystem;

/**
 * Command for controlling the launcher using a PID loop.
 * <p>
 * This command sets a target speed for the launcher and continuously monitors
 * the current speed of the launcher subsystem.
 */
public class LauncherSetVelocityPIDCmd extends Command {

  private final LauncherSubsystem launcherSub;
  private final double referenceVelocity;

  /**
   * Constructs a new LauncherSetVelocityPID.
   * 
   * @param launcherSub       The launcher subsystem that this command will
   *                          control.
   * @param referenceVelocity A supplier that provides the target speed for the
   *                          launcher, in RPM.
   */
  public LauncherSetVelocityPIDCmd(LauncherSubsystem launcherSub, double referenceVelocity) {
    this.launcherSub = launcherSub;
    this.referenceVelocity = referenceVelocity;

    addRequirements(launcherSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherSub.setReferenceVelocity(referenceVelocity);
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
