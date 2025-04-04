// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants.ReefConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToPosePIDCmd extends Command {
  private final Drivetrain driveSub;
  private final PIDController rotationController = new PIDController(AutoConstants.kPThetaController,
      AutoConstants.kIThetaController, AutoConstants.kDThetaController),
      translationYController = new PIDController(AutoConstants.kPTranslationController,
          AutoConstants.kITranslationController, AutoConstants.kDTranslationController),
      translationXController = new PIDController(AutoConstants.kPTranslationController,
          AutoConstants.kITranslationController, AutoConstants.kDTranslationController);

  private Supplier<Pose2d> targetPose;
  private Pose2d initialTargetPose;
  private double targetX, targetY, targetRadians;
  private boolean dynamicPose;

  /** Creates a new MoveToTargetPoseCmd. */
  public MoveToPosePIDCmd(Drivetrain driveSub, Supplier<Pose2d> targetPose, boolean dynamicPose) {
    this.driveSub = driveSub;
    addRequirements(driveSub);

    this.targetPose = targetPose;
    this.dynamicPose = dynamicPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initialTargetPose = targetPose.get();

    // Run calculations once to ensure that the three bang bang controllers have
    // setpoints
    Pose2d currentPose = driveSub.getPose();
    Logger.recordOutput("Odometry/MoveToTargetPose/CurrentPose",
        currentPose);
    Logger.recordOutput("Odometry/MoveToTargetPose/IsDynamicPose",
        dynamicPose);

    // Set target based on whether the target pose is dynamic
    if (dynamicPose) {
      targetX = targetPose.get().getX();
      targetY = targetPose.get().getY();
      targetRadians = targetPose.get().getRotation().getRadians();
      Logger.recordOutput("Odometry/MoveToTargetPose/TargetPose",
          targetPose.get());

      // If target pose is not dynamic, always use initial target pose
    } else {
      targetX = initialTargetPose.getX();
      targetY = initialTargetPose.getY();
      targetRadians = initialTargetPose.getRotation().getRadians();
      Logger.recordOutput("Odometry/MoveToTargetPose/TargetPose",
          initialTargetPose);
    }

    translationXController.setSetpoint(targetX);
    rotationController.setTolerance(ReefConstants.kToleranceMetersFromSpot);

    translationYController.setSetpoint(targetY);
    rotationController.setTolerance(ReefConstants.kToleranceMetersFromSpot);

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setSetpoint(targetRadians);
    rotationController.setTolerance(ReefConstants.kToleranceRadiansFromSpot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = driveSub.getPose();
    Logger.recordOutput("Odometry/MoveToTargetPose/CurrentPose",
        currentPose);

    // Set target based on whether the target pose is dynamic
    if (dynamicPose) {
      targetX = targetPose.get().getX();
      targetY = targetPose.get().getY();
      targetRadians = targetPose.get().getRotation().getRadians();
      translationXController.setSetpoint(targetX);
      translationYController.setSetpoint(targetY);
      rotationController.setSetpoint(targetRadians);
      Logger.recordOutput("Odometry/MoveToTargetPose/TargetPose",
          targetPose.get());

      // If target pose is not dynamic, always use initial target pose
    } else {
      Logger.recordOutput("Odometry/MoveToTargetPose/TargetPose",
          initialTargetPose);
    }

    // Make adjustments to the robot

    // Reverse direction if on red alliance

    // Convert calculated value to velocity
    double xVel = translationXController.calculate(targetX) * AutoConstants.kMaxSpeedMetersPerSecond;
    double yVel = translationYController.calculate(targetY) * AutoConstants.kMaxSpeedMetersPerSecond;
    double rotVel = rotationController.calculate(targetRadians) * AutoConstants.kMaxAngularSpeedRadiansPerSecond;

    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Red) {
        xVel *= -1;
        yVel *= -1;
      }
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel, yVel, rotVel);
    driveSub.runVelocityFieldRelative(chassisSpeeds);

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
    return false;
  }
}
