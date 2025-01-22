// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrive extends Command {
  private Timer timer = new Timer();
  private Drivetrain driveSub;
  double xVel;
  double yVel;
  double omega;
  double seconds;

  /** Creates a new AutoDrive. */
  public AutoDrive(Drivetrain driveSub, double secs, double x, double y, double ohm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSub = driveSub;
    addRequirements(driveSub);
    xVel = x * DriveConstants.kMaxSpeedMetersPerSecond;
    yVel = y * DriveConstants.kMaxSpeedMetersPerSecond;
    omega = ohm * DriveConstants.kMaxAngularSpeed;
    seconds = secs;
  }

  public AutoDrive(Drivetrain driveSub) {
    this(driveSub, 3, 0.5, 0, 0);
  }

  public AutoDrive(Drivetrain driveSub, double x, double y, double ohm) {
    this(driveSub, 3, x, y, ohm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < seconds) {
      driveSub.runVelocityFieldRelative(new ChassisSpeeds(xVel, yVel, omega));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.runVelocityFieldRelative(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= seconds;
  }
}
