package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * Command for controlling the shooter using a PID loop.
 * <p>
 * This command sets a target speed for the shooter and continuously monitors
 * the current speed of the shooter subsystem. It runs until explicitly
 * interrupted.
 */
public class ShooterPID extends Command {

  private final ShooterSubsystem shooterSub;
  private final DoubleSupplier goalSpeed;

  /**
   * Constructs a new ShooterPID command.
   * 
   * @param shooterSub The shooter subsystem that this command will control.
   * @param goalSpeed  A supplier that provides the target speed for the shooter.
   */
  public ShooterPID(ShooterSubsystem shooterSub, DoubleSupplier goalSpeed) {
    this.shooterSub = shooterSub;
    this.goalSpeed = goalSpeed;
    addRequirements(shooterSub);
  }

  /**
   * Initializes the ShooterPID command by setting the reference speed for the
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
   * Executes the ShooterPID command.
   * <p>
   * This method is called repeatedly while the command is scheduled.
   * It reports the current speed of the shooter to the SmartDashboard.
   */
  @Override
  public void execute() {
    double currentSpeed = shooterSub.getShooterVelocity();
    SmartDashboard.putNumber("Shooter/CurrentSpeed", currentSpeed);
  }

  /**
   * Ends the ShooterPID command by stopping the shooter.
   * <p>
   * This method is called once when the command ends or is interrupted.
   * 
   * @param interrupted True if the command was interrupted, false if it completed
   *                    normally.
   */
  @Override
  public void end(boolean interrupted) {
    shooterSub.setReferenceSpeed(0); // Stop the shooter when the command ends
    // System.out.println("ShooterPID ended. Interrupted: " + interrupted);
  }

  /**
   * Determines if the ShooterPID command is finished.
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
