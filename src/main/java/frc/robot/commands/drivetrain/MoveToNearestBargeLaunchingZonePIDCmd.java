// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.DriveConstants.LaunchingDistances;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToNearestBargeLaunchingZonePIDCmd extends Command {
  private final Drivetrain driveSub;
  private final PIDController translationController = new PIDController(AutoConstants.kPTranslationController,
      AutoConstants.kITranslationController, AutoConstants.kDTranslationController);
  private final PIDController rotationController = new PIDController(AutoConstants.kPThetaController,
      AutoConstants.kIThetaController, AutoConstants.kDThetaController);

  /** Creates a new MoveToNearestBargeLaunchingZoneCmd. */
  public MoveToNearestBargeLaunchingZonePIDCmd(
      Drivetrain driveSub) {
    this.driveSub = driveSub;
    addRequirements(driveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translationController.setTolerance(LaunchingDistances.kToleranceDistanceFromBarge.in(Meters));
    rotationController.setTolerance(LaunchingDistances.kToleranceAngleFromBarge.in(Radians));
  }

  @Override
  public void execute() {
    Pose2d currentPose = driveSub.getPose();

    // Calculate target pose
    boolean onBlueSide = driveSub.isOnBlueSide();

    // Calculate target translation
    // (0,0) is ALWAYS on the blue alliance side
    Distance targetX = FieldConstants.kBargeX.plus(LaunchingDistances.kDistanceFromBarge.times((onBlueSide ? -1 : 1)));

    // Calculate target rotation based on side of field that robot is currently on
    Angle targetAngle = Radians.of(onBlueSide ? Math.PI : 0);

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/CurrentPose",
        currentPose);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/TargetPose",
        new Pose2d(targetX.in(Meters), currentPose.getY(),
            Rotation2d.fromRadians(targetAngle.in(Radians))));

    LinearVelocity xVel = MetersPerSecond.of(
        MathUtil.clamp(translationController.calculate(currentPose.getX(), targetX.in(Meters)),
            -AutoConstants.kMaxSpeed.in(MetersPerSecond),
            AutoConstants.kMaxSpeed.in(MetersPerSecond)));

    // Invert the direction if robot is on red alliance
    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Red) {
        xVel = xVel.times(-1);
      }
    }

    Angle currentRotation = Radians.of(currentPose.getRotation().getRadians());

    AngularVelocity rotVel = RadiansPerSecond.of(MathUtil.clamp(
        rotationController.calculate(
            currentRotation.in(Radians),
            targetAngle.times(Math.signum(currentRotation.in(Radians))).in(Radians)

        ),
        -AutoConstants.kMaxAngularSpeed.in(RadiansPerSecond),
        AutoConstants.kMaxAngularSpeed.in(RadiansPerSecond)

    ));

    // Set drivetrain to run at calculated velocity
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xVel.in(MetersPerSecond), 0, rotVel.in(RadiansPerSecond));
    driveSub.runVelocityFieldRelative(chassisSpeeds);

    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/PID/OutputVelocityX", xVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/PID/OutputVelocityRotation", rotVel);
    Logger.recordOutput("Odometry/MoveToNearestBargeLaunchingZone/PID/OutputChassisSpeeds", chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}