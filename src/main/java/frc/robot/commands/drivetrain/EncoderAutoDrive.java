// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants.EncoderAutoDriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EncoderAutoDrive extends Command {
  private final Drivetrain driveSub;
  private final Distance netdistance;
  private final LinearVelocity xVel, yVel;

  private SwerveModulePosition[] initialModulePositions;

  /**
   * Attempts to move the robot in a straight line given a x and y displacement.
   * 
   * @param driveSub    Drivetrain subsystem
   * @param netVelocity Velocity (Meters-per-second)
   * @param xDis        Desired x-displacement in meters (robot-oriented)
   * @param yDis        Desired y-displacement in meters (robot-oriented)
   */
  public EncoderAutoDrive(
      Drivetrain driveSub,
      LinearVelocity netVelocity,
      Distance xDis, Distance yDis) {

    this.driveSub = driveSub;

    addRequirements(driveSub);

    // Get the total net distance
    netdistance = Meters.of(Math.sqrt(Math.pow(xDis.in(Meters), 2) + Math.pow(yDis.in(Meters), 2)));

    // Angle (degrees): {Moving left < 0 < Moving right}
    Angle heading = Degrees.of(xDis.in(Meters) == 0 ? 0 : Math.atan(xDis.in(Meters) / yDis.in(Meters)));

    // Calculate required x-velocity to meet net velocity
    xVel = MetersPerSecond.of(Math.sin(heading.in(Degrees)) * netVelocity.in(MetersPerSecond));

    // Calculate required y-velocity to meet net velocity
    yVel = MetersPerSecond.of(Math.cos(heading.in(Degrees)) * netVelocity.in(MetersPerSecond));

  }

  /**
   * Attempts to move the robot in a straight line given a x and y displacement.
   * Using this constructor will default to robot moving out of zone.
   * 
   * @param driveSub Drivetrain subsystem
   */
  public EncoderAutoDrive(Drivetrain driveSub) {
    // Move out of zone auto
    this(driveSub, EncoderAutoDriveConstants.kLeaveZoneVelocity, EncoderAutoDriveConstants.kLeaveZoneDistance,
        Meters.zero());
  }

  @Override
  public void initialize() {
    initialModulePositions = driveSub.getModulePositions(); // Save the initial recorded encoder values of the driving
                                                            // wheels
  }

  @Override
  public void execute() {
    driveSub.runVelocityRobotRelative(new ChassisSpeeds(xVel.in(MetersPerSecond), yVel.in(MetersPerSecond), 0));
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
    return currentAverageDistance >= netdistance.in(Meters);
  }
}
