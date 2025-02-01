package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    shooterSub.setReferenceSpeed(speed); // Set the goal speed for the shooter
  }

  @Override
  public void execute() {
    double currentSpeed = shooterSub.getShooterVelocity();
    SmartDashboard.putNumber("Shooter/CurrentSpeed", currentSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.setReferenceSpeed(0); // Stop the shooter when the command ends
    // System.out.println("ShooterPID ended. Interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false; // Run indefinitely until interrupted
  }
}