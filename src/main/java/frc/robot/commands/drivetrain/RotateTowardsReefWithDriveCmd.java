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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DriveConstants.ReefConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.utils.PoseHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateTowardsReefWithDriveCmd extends Command {
  private Supplier<Double> forwardsBackwardsSupplier;

  private final Drivetrain driveSub;
  private final BangBangController rotationController = new BangBangController(ReefConstants.kToleranceRadiansFromSpot),
      translationControllerY = new BangBangController(ReefConstants.kToleranceRadiansFromSpot),
      translationControllerX = new BangBangController(ReefConstants.kToleranceMetersFromSpot);

  private double targetX, targetY, targetRadians;

  /** Creates a new PathfindToReefSpotCmd. */
  public RotateTowardsReefWithDriveCmd(
      Drivetrain driveSub, Supplier<Double> forwardsBackwardsSupplier) {
    this.driveSub = driveSub;
    this.forwardsBackwardsSupplier = forwardsBackwardsSupplier;
    addRequirements(driveSub);
  }

  public RotateTowardsReefWithDriveCmd(Drivetrain driveSub) {
    this(driveSub, () -> 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Pose2d currentPose = driveSub.getPose();

    // // Get the closest reef spot
    // ArrayList<Pose3d> reefTagPoses = new ArrayList<Pose3d>();
    // for (int tagId : ReefConstants.kReefTagIds) {
    // Optional<Pose3d> optionalTagPose =
    // FieldConstants.kfieldLayout.getTagPose(tagId);
    // if (optionalTagPose.isEmpty()) {
    // continue;
    // }
    // reefTagPoses.add(optionalTagPose.get());
    // }

    // ArrayList<Pose2d> reefSpots = new ArrayList<Pose2d>();
    // for (Pose3d reefTagPose : reefTagPoses) {
    // reefSpots.add(PoseHelpers.toPose2d(reefTagPose).transformBy(ReefConstants.kOffsetFromTag));
    // }

    // Pose2d targetReefSpot = reefSpots.get(0);
    // double nearestDistance = PoseHelpers.distanceBetween(currentPose,
    // targetReefSpot);
    // for (Pose2d reefSpot : reefSpots) {
    // double distance = PoseHelpers.distanceBetween(currentPose, reefSpot);
    // if (distance < nearestDistance) {
    // targetReefSpot = reefSpot;
    // nearestDistance = distance;
    // }
    // }

    // Logger.recordOutput("Odometry/MoveToNearestReefSpot/CurrentPose",
    // currentPose);
    // Logger.recordOutput("Odometry/MoveToNearestReefSpot/TargetPose",
    // targetReefSpot);

    // targetRadians = targetReefSpot.getRotation().getRadians();

    // // Make adjustments to the robot
    // double currentRotation = currentPose.getRotation().getRadians();
    // rotationController.calculate(MathUtil.angleModulus(currentRotation -
    // targetRadians), 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = driveSub.getPose();

    // Get the closest reef spot
    ArrayList<Pose3d> reefTagPoses = new ArrayList<Pose3d>();
    for (int tagId : ReefConstants.kReefTagIds) {
      Optional<Pose3d> optionalTagPose = FieldConstants.kfieldLayout.getTagPose(tagId);
      if (optionalTagPose.isEmpty()) {
        continue;
      }
      reefTagPoses.add(optionalTagPose.get());
    }

    ArrayList<Pose2d> reefSpots = new ArrayList<Pose2d>();
    for (Pose3d reefTagPose : reefTagPoses) {
      reefSpots.add(PoseHelpers.toPose2d(reefTagPose));
    }

    Pose2d targetReefTagPose = reefSpots.get(0);
    double nearestDistance = PoseHelpers.distanceBetween(currentPose, targetReefTagPose);
    for (Pose2d reefSpot : reefSpots) {
      double distance = PoseHelpers.distanceBetween(currentPose, reefSpot);
      if (distance < nearestDistance) {
        targetReefTagPose = reefSpot;
        nearestDistance = distance;
      }
    }

    Logger.recordOutput("Odometry/MoveToNearestReefSpot/CurrentPose",
        currentPose);
    Logger.recordOutput("Odometry/MoveToNearestReefSpot/TargetReefTagPose",
        targetReefTagPose);

    double angle = targetReefTagPose.getRotation().getRadians();

    // define a normal line that extends out of the tag
    double normalSlope = Math.tan(angle);
    double normalYInt = targetReefTagPose.getY() - normalSlope * targetReefTagPose.getX();

    // define a perpendiculat line intersecting the robot
    double botSlope = 1 / normalSlope;
    double botYInt = currentPose.getY() - botSlope * currentPose.getX();

    targetX = (normalYInt - botYInt) / (botSlope - normalSlope);
    targetY = normalSlope * targetX + normalYInt;
    targetRadians = targetReefTagPose.getRotation().getRadians();

    // Make adjustments to the robot
    double directionRot = 0;
    double directionX = 0;
    double directionY = 0;

    double currentRotation = currentPose.getRotation().getRadians();

    // Bang bang controller returns 0 or 1
    // Multiply calculated output by 2 and subtract 1 to get -1 or 1
    // Offset the rotation such that the setpoint is always "0". This rids of
    // wrap-around issues.
    directionRot = (rotationController.calculate(MathUtil.angleModulus(currentRotation - targetRadians), 0) * 2) - 1;
    if (rotationController.atSetpoint()) {
      directionRot = 0;
    }
    directionX = (translationControllerX.calculate(currentPose.getX(), targetX) * 2) - 1;
    directionY = (translationControllerY.calculate(currentPose.getY(), targetY) * 2) - 1;
    if (rotationController.atSetpoint()) {
      directionRot = 0;
    }
    if (translationControllerX.atSetpoint()) {
      directionX = 0;
    }
    if (translationControllerY.atSetpoint()) {
      directionY = 0;
    }

    double forwardsBackwardsVel = forwardsBackwardsSupplier.get() * DriveConstants.kMaxSpeedMetersPerSecond;

    // Convert calculated value to velocity
    double xVel = (directionX * DriveConstants.kMaxSpeedMetersPerSecond)
        - (forwardsBackwardsVel * Math.cos(targetRadians));
    double yVel = (directionY * DriveConstants.kMaxSpeedMetersPerSecond)
        - (forwardsBackwardsVel * Math.sin(targetRadians));

    double rotVel = DriveConstants.kBangBangRotationalVelocityRadiansPerSecond * directionRot;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel, yVel, rotVel);
    driveSub.runVelocityFieldRelative(chassisSpeeds);

    Logger.recordOutput("Odometry/MoveToNearestReefSpot/InputForwardsBackwardsVel", forwardsBackwardsVel);
    Logger.recordOutput("Odometry/MoveToNearestReefSpot/OutputDirectionRotation", directionRot);
    Logger.recordOutput("Odometry/MoveToNearestReefSpot/OutputVelocityRotation", rotVel);
    Logger.recordOutput("Odometry/MoveToNearestReefSpot/OutputChassisSpeeds", chassisSpeeds);

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
