// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * The DriveCmd class is a command that controls the drivetrain using joystick
 * inputs.
 */
public class DriveCmd extends Command {

  // Reference to the drivetrain subsystem
  private Drivetrain driveSub;

  // Suppliers for joystick inputs
  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;
  private Supplier<Double> omegaSupplier;
  public boolean gyroDisconnected;
  Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  /**
   * Creates a new DriveCmd.
   * 
   * @param driveSub      The drivetrain subsystem used by this command.
   * @param xSupplier     Supplier for the x-axis joystick input.
   * @param ySupplier     Supplier for the y-axis joystick input.
   * @param omegaSupplier Supplier for the omega (rotation) joystick input.
   */
  public DriveCmd(Drivetrain driveSub, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
      Supplier<Double> omegaSupplier) {
    this.driveSub = driveSub;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;

    // Declare subsystem dependencies
    addRequirements(driveSub);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    // No initialization needed for this command
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * This method reads the joystick inputs and sets the drivetrain speeds
   * accordingly.
   */
  @Override
  public void execute() {
    double xVel = xSupplier.get() * DriveConstants.kMaxSpeedMetersPerSecond;
    double yVel = ySupplier.get() * DriveConstants.kMaxSpeedMetersPerSecond;
    double omega = omegaSupplier.get() * DriveConstants.kMaxAngularSpeed;
    driveSub.runVelocity(new ChassisSpeeds(xVel, yVel, omega));
  }

  /**
   * Called once the command ends or is interrupted.
   * This method is empty because no cleanup is necessary.
   */
  @Override
  public void end(boolean interrupted) {
    // No cleanup necessary
  }

  /**
   * Returns true when the command should end.
   * This command continues running until explicitly interrupted.
   * 
   * @return false to indicate the command should not end.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}