// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DriveConstants.LaunchingDistances;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.PoseHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToNearestBargeLaunchingZoneBangBangCmd extends Command {
  private final Drivetrain driveSub;
  private final BangBangController translationController = new BangBangController(
      LaunchingDistances.kToleranceDistanceFromBarge.in(Meters));
  private final BangBangController rotationController = new BangBangController(
      LaunchingDistances.kToleranceAngleFromBarge.in(Radians));

  private Distance targetX;
  private Angle targetAngle;

  private boolean translationFinish = false, rotationFinish = false;

  /** Creates a new MoveToNearestBargeLaunchingZoneCmd. */
  public MoveToNearestBargeLaunchingZoneBangBangCmd(
      Drivetrain driveSub) {
    this.driveSub = driveSub;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translationFinish = false;
    rotationFinish = false;
  }

  @Override
  public void execute() {
    Pose2d currentPose = driveSub.getPose();

    // Calculate target pose
    boolean onBlueSide = PoseHelpers.isOnBlueSide(currentPose);

    // Calculate target translation
    // (0,0) is ALWAYS on the blue alliance side
    targetX = FieldConstants.kBargeX.plus(LaunchingDistances.kDistanceFromBarge.times((onBlueSide ? -1 : 1)));

    // Calculate target rotation based on side of field that robot is currently on
    targetAngle = Radians.of(onBlueSide ? Math.PI : 0);

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/CurrentPose",
        currentPose);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TargetPose",
        new Pose2d(targetX.in(Meters), currentPose.getY(),
            Rotation2d.fromRadians(targetAngle.in(Radians))));

    // Make adjustments to the robot
    double directionX = 0;
    double directionRot = 0;

    // Calculate translation from bang bang controller
    if (!translationFinish) {
      // Bang bang controller returns 0 or 1
      // Multiply calculated output by 2 and subtract 1 to get -1 or 1
      directionX = (translationController.calculate(currentPose.getX(), targetX.in(Meters)) * 2) - 1;

      // Reverse direction if on red alliance
      if (DriverStation.getAlliance().isPresent()) {
        Alliance alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Red) {
          directionX *= -1;
        }
      }

      translationFinish = translationController.atSetpoint();
    }

    // Calculate rotation from bang bang controller
    if (!rotationFinish) {
      Rotation2d currentRotation = currentPose.getRotation();

      // Bang bang controller returns 0 or 1
      // Multiply calculated output by 2 and subtract 1 to get -1 or 1
      directionRot = (rotationController.calculate(
          currentRotation.getRadians(),
          targetAngle.times(Math.signum(currentRotation.getRadians())).in(Radians)) * 2) - 1;

      rotationFinish = rotationController.atSetpoint();
    }

    // Convert calculated value to velocity
    double xVel = DriveConstants.kBangBangTranslationalVelocity.in(MetersPerSecond) * directionX;
    double rotVel = DriveConstants.kBangBangRotationalVelocity.in(RadiansPerSecond) * directionRot;

    // Set drivetrain to run at calculated velocity
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel, 0, rotVel);
    driveSub.runVelocityFieldRelative(chassisSpeeds);

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/BangBang/OutputDirectionX", directionX);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/BangBang/OutputDirectionRotation", directionRot);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/BangBang/OutputVelocityX", xVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/BangBang/OutputVelocityRotation", rotVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/BangBang/OutputChassisSpeeds", chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return translationFinish && rotationFinish;
  }
}