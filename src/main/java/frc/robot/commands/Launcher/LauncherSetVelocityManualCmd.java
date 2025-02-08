package frc.robot.commands.Launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher.LauncherSubsystem;

/**
 * Command to set the speed of the shooter subsystem.
 * <p>
 * This command sets the shooter speed based on a provided value. It runs
 * indefinitely until interrupted, during which it continuously updates the
 * speed.
 */
public class LauncherSetVelocityManualCmd extends Command {

  private final LauncherSubsystem launcherSub;
  private final DoubleSupplier velocity;

  /**
   * Constructs a new SetLauncherSpeed command.
   * 
   * @param launcherSub The shooter subsystem that this command will control.
   * @param speed       A supplier that provides the speed value to set the
   *                    shooter
   *                    to.
   */
  public LauncherSetVelocityManualCmd(LauncherSubsystem launcherSub, DoubleSupplier velocity) {
    this.launcherSub = launcherSub;
    this.velocity = velocity;
    addRequirements(launcherSub);
  }

  /**
   * Initializes the SetLauncherSpeed command.
   * <p>
   * This method is called once when the command is first scheduled. In this case,
   * no special initialization is needed, so the method is empty.
   */
  @Override
  public void initialize() {
  }

  /**
   * Executes the SetLauncherSpeed command.
   * <p>
   * This method is called repeatedly while the command is scheduled.
   * It sets the speed of the shooter subsystem to the value provided by the speed
   * supplier.
   */
  @Override
  public void execute() {
    double velocityValue = velocity.getAsDouble();
    launcherSub.setVelocity(velocityValue);
  }

  /**
   * Ends the SetLauncherSpeed command.
   * <p>
   * This method is called once when the command ends or is interrupted.
   * It stops the shooter by setting its speed to 0.
   * 
   * @param interrupted True if the command was interrupted, false if it completed
   *                    normally.
   */
  @Override
  public void end(boolean interrupted) {
    launcherSub.stopLauncher();
  }

  /**
   * Determines if the SetLauncherSpeed command is finished.
   * <p>
   * Since this command runs indefinitely, it always returns false.
   * 
   * @return False, as the command does not finish on its own.
   */
  @Override
  public boolean isFinished() {
    return false; // Run indefinitely until interrupted
  }
}
