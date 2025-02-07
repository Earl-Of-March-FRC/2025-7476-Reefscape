package frc.robot.commands.Launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher.LauncherSubsystem;

/**
 * Command for controlling the shooter using a PID loop.
 * <p>
 * This command sets a target speed for the shooter and continuously monitors
 * the current speed of the shooter subsystem. It runs until explicitly
 * interrupted.
 */
public class LauncherPID extends Command {

  private final LauncherSubsystem shooterSub;
  private final DoubleSupplier goalSpeed;

  /**
   * Constructs a new LauncherPID command.
   * 
   * @param shooterSub The shooter subsystem that this command will control.
   * @param goalSpeed  A supplier that provides the target speed for the shooter.
   */
  public LauncherPID(LauncherSubsystem shooterSub, DoubleSupplier goalSpeed) {
    this.shooterSub = shooterSub;
    this.goalSpeed = goalSpeed;
    addRequirements(shooterSub);
  }

  /**
   * Initializes the LauncherPID command by setting the reference speed for the
   * shooter.
   * <p>
   * This method is called once when the command is first scheduled.
   */
  @Override
  public void initialize() {
    double speed = goalSpeed.getAsDouble();
    shooterSub.setReferenceSpeed(speed); // Set the goal speed for the shooter
  }

  /**
   * Executes the LauncherPID command.
   * <p>
   * This method is called repeatedly while the command is scheduled.
   * It reports the current speed of the shooter to the SmartDashboard.
   */
  @Override
  public void execute() {
    double currentSpeed = shooterSub.getLauncherVelocity();
    SmartDashboard.putNumber("Launcher/CurrentSpeed", currentSpeed);
  }

  /**
   * Ends the LauncherPID command by stopping the shooter.
   * <p>
   * This method is called once when the command ends or is interrupted.
   * 
   * @param interrupted True if the command was interrupted, false if it completed
   *                    normally.
   */
  @Override
  public void end(boolean interrupted) {
    shooterSub.setReferenceSpeed(0); // Stop the shooter when the command ends
    // System.out.println("LauncherPID ended. Interrupted: " + interrupted);
  }

  /**
   * Determines if the LauncherPID command is finished.
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
