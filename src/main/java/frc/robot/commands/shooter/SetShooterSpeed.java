package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class SetShooterSpeed extends Command {
  private final ShooterSubsystem shooterSub;
  private final DoubleSupplier speed;

  public SetShooterSpeed(ShooterSubsystem shooterSub, DoubleSupplier speed) {
    this.shooterSub = shooterSub;
    this.speed = speed;
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
    System.out.println("SetShooterSpeed command started");
  }

  @Override
  public void execute() {
    double speedValue = speed.getAsDouble();
    shooterSub.setShooterSpeed(speedValue);
    System.out.println("Setting shooter speed to: " + speedValue);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.setShooterSpeed(0);
    System.out.println("SetShooterSpeed command ended");
  }

  @Override
  public boolean isFinished() {
    return false; // Run indefinitely until interrupted
  }
}