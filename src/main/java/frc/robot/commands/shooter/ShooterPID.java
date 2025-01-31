package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterPID extends Command {
  private final ShooterSubsystem shooterSub;
  private final DoubleSupplier goalSpeed;

  public ShooterPID(ShooterSubsystem shooterSub, DoubleSupplier goalSpeed) {
    this.shooterSub = shooterSub;
    this.goalSpeed = goalSpeed;
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
    double speed = goalSpeed.getAsDouble();
    shooterSub.setReferenceSpeed(speed);
    System.out.println("ShooterPID command started with goal speed: " + speed);
  }

  @Override
  public void execute() {
    // Optional: Add additional logic or logging here
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.setReferenceSpeed(0);
    System.out.println("ShooterPID command ended");
  }

  @Override
  public boolean isFinished() {
    return false; // Run indefinitely until interrupted
  }
}