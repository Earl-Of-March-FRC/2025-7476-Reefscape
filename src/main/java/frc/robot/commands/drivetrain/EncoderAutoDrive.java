// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EncoderAutoDrive extends Command {
  private final Drivetrain driveSub;
  private final double netdistance, xVel, yVel;

  private SwerveModulePosition[] initialModulePositions;

  /**
   * Attempts to move the robot in a straight line given a x and y displacement.
   * 
   * @param driveSub
   * @param netVelocity Velocity (Meters-per-second)
   * @param xDis        Desired x-displacement in meters (robot-oriented)
   * @param yDis        Desired y-displacement in meters (robot-oriented)
   */
  public EncoderAutoDrive(
      Drivetrain driveSub,
      double netVelocity,
      double xDis, double yDis) {

    this.driveSub = driveSub;

    addRequirements(driveSub);

    // Get the total net distance
    netdistance = Math.sqrt(Math.pow(xDis, 2) + Math.pow(yDis, 2));

    double heading = (yDis == 0 ? 0 : Math.tan(xDis / yDis)); // Angle (degrees): {Moving left < 0 < Moving right}
    xVel = Math.asin(heading) * netVelocity; // Calculate required x-velocity to meet net velocity
    yVel = Math.acos(heading) * netVelocity; // Calculate required y-velocity to meet net velocity

  }

  @Override
  public void initialize() {
    initialModulePositions = driveSub.getModulePositions(); // Save the initial recorded encoder values of the driving
                                                            // wheels
  }

  @Override
  public void execute() {
    driveSub.runVelocity(new ChassisSpeeds(xVel, yVel, 0));
  }

  @Override
  public boolean isFinished() {
    double currentAverageDistance = 0; // Stores average position change of all driving motors
    SwerveModulePosition[] currentModulePositions = driveSub.getModulePositions(); // Get current module positions
    for (int i = 0; i < initialModulePositions.length; i++) { // Iterate
      double initialPosition = initialModulePositions[i].distanceMeters;
      double currentPosition = currentModulePositions[i].distanceMeters;
      /*
       * Get the position difference (regardless of positive or negative) and add to
       * average
       */
      currentAverageDistance += Math.abs(currentPosition - initialPosition) /
          initialModulePositions.length;
    }
    return currentAverageDistance >= netdistance;
  }
}
