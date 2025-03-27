// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ReefConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.PoseHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToTargetPoseCmd extends Command {
  private final Drivetrain driveSub;
  private final BangBangController translationXController = new BangBangController(
      ReefConstants.kToleranceMetersFromSpot);
  private final BangBangController translationYController = new BangBangController(
      ReefConstants.kToleranceMetersFromSpot);
  private final BangBangController rotationController = new BangBangController(ReefConstants.kToleranceRadiansFromSpot);

  private Pose2d targetPose;
  private double targetX, targetY, targetRadians;

  private boolean translationXFinish = false, translationYFinish = false, rotationFinish = false;

  /** Creates a new MoveToTargetPoseCmd. */
  public MoveToTargetPoseCmd(Drivetrain driveSub, Pose2d targetPose) {
    this.driveSub = driveSub;
    addRequirements(driveSub);

    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translationXFinish = false;
    translationYFinish = false;
    rotationFinish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = driveSub.getPose();

    // Get the current target pose
    targetX = targetPose.getX();
    targetY = targetPose.getY();
    targetRadians = targetPose.getRotation().getRadians();

    Logger.recordOutput("Odometry/MoveToTargetPose/CurrentPose",
        currentPose);
    Logger.recordOutput("Odometry/MoveToTargetPose/TargetPose",
        targetPose);

    // Make adjustments to the robot
    double directionX = 0;
    double directionY = 0;
    double directionRot = 0;

    if (!translationXFinish) {
      // Bang bang controller returns 0 or 1
      // Multiply calculated output by 2 and subtract 1 to get -1 or 1
      directionX = (translationXController.calculate(currentPose.getX(), targetX) * 2) - 1;

      // Reverse direction if on red alliance
      if (DriverStation.getAlliance().isPresent()) {
        Alliance alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Red) {
          directionX *= -1;
        }
      }

      translationXFinish = translationXController.atSetpoint();
    }

    if (!translationYFinish) {
      // Bang bang controller returns 0 or 1
      // Multiply calculated output by 2 and subtract 1 to get -1 or 1
      directionY = (translationYController.calculate(currentPose.getY(), targetY) * 2) - 1;

      // Reverse direction if on red alliance
      if (DriverStation.getAlliance().isPresent()) {
        Alliance alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Red) {
          directionY *= -1;
        }
      }

      translationYFinish = translationYController.atSetpoint();
    }

    if (!rotationFinish) {
      double currentRotation = currentPose.getRotation().getRadians();

      // Bang bang controller returns 0 or 1
      // Multiply calculated output by 2 and subtract 1 to get -1 or 1
      // Offset the rotation such that the setpoint is always "0". This rids of
      // wrap-around issues.
      directionRot = (rotationController.calculate(MathUtil.angleModulus(currentRotation - targetRadians), 0) * 2) - 1;
      rotationFinish = rotationController.atSetpoint();
    }

    // Convert calculated value to velocity
    double xVel = DriveConstants.kBangBangTranslationalVelocityMetersPerSecond * directionX;
    double yVel = DriveConstants.kBangBangTranslationalVelocityMetersPerSecond * directionY;
    double rotVel = DriveConstants.kBangBangRotationalVelocityRadiansPerSecond * directionRot;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel, yVel, rotVel);
    driveSub.runVelocityFieldRelative(chassisSpeeds);

    Logger.recordOutput("Odometry/MoveToTargetPose/OutputDirectionX", directionX);
    Logger.recordOutput("Odometry/MoveToTargetPose/OutputDirectionY", directionY);
    Logger.recordOutput("Odometry/MoveToTargetPose/OutputDirectionRotation", directionRot);
    Logger.recordOutput("Odometry/MoveToTargetPose/OutputVelocityX", xVel);
    Logger.recordOutput("Odometry/MoveToTargetPose/OutputVelocityY", yVel);
    Logger.recordOutput("Odometry/MoveToTargetPose/OutputVelocityRotation", rotVel);
    Logger.recordOutput("Odometry/MoveToTargetPose/OutputChassisSpeeds", chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return translationXFinish && translationYFinish && rotationFinish;
  }
}
