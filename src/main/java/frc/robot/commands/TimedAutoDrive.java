// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This command just sends the robot in one direction for a specified amount of seconds (default forward for 3 seconds)

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TimedAutoDrive extends Command {
  // Timer to keep track of the elapsed time
  private Timer timer = new Timer();
  
  // Reference to the drivetrain subsystem
  private Drivetrain driveSub;
  
  // Desired velocities and duration for the autonomous drive
  double xVel;
  double yVel;
  double omega;
  double seconds;

  /** 
   * Creates a new TimedAutoDrive command.
   * 
   * @param driveSub The drivetrain subsystem used by this command.
   * @param secs The duration for which the robot should drive.
   * @param x The desired x-axis velocity.
   * @param y The desired y-axis velocity.
   * @param ohm The desired rotational velocity.
   */
  public TimedAutoDrive(Drivetrain driveSub, double secs, double x, double y, double ohm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSub = driveSub;
    addRequirements(driveSub);
    
    // Set the desired velocities and duration
    xVel = x * AutoConstants.kMaxSpeedMetersPerSecond;
    yVel = y * AutoConstants.kMaxSpeedMetersPerSecond;
    omega = ohm * AutoConstants.kMaxAngularSpeed;
    seconds = secs;
  }

  public TimedAutoDrive(Drivetrain driveSub) {
    this(driveSub, 3, 1, 0, 0);
  }

  public TimedAutoDrive(Drivetrain driveSub, double x, double y, double ohm) {
    this(driveSub, 3, x, y, ohm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * Sets the drivetrain velocities.
   */
  @Override
  public void execute() {
    if (timer.get() < seconds) {
      driveSub.runVelocityFieldRelative(new ChassisSpeeds(xVel, yVel, omega));
    }
  }

  /**
   * Called once the command ends or is interrupted.
   * Stops the drivetrain.
   */
  @Override
  public void end(boolean interrupted) {
    driveSub.runVelocityFieldRelative(new ChassisSpeeds(0, 0, 0));
  }

  /**
   * Returns true when the command should end.
   * The command ends when the specified duration has elapsed.
   * 
   * @return true if the specified duration has elapsed, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return timer.get() >= seconds;
  }
}
