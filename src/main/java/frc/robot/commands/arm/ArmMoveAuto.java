// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmMoveAuto extends Command {

  private ArmSubsystem armSub;
  private ProfiledPIDController pidController;

  /** Creates a new ShoulderMoveAuto. */
  public ArmMoveAuto(ArmSubsystem armSub, double goalAngle) {

    this.armSub = armSub;

    // creates Profiled PID Controller
    pidController = new ProfiledPIDController(
        // sets PID gains
        ArmConstants.kPArmController,
        ArmConstants.kIArmController,
        ArmConstants.kDArmController,

        // sets constraints
        ArmConstants.kArmControllerConstraints);

    // sets goal and tolerance for controller
    pidController.setGoal(goalAngle);
    pidController.setTolerance(1);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset(armSub.getArmPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setArmSpeed(pidController.calculate(armSub.getArmPosition()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atGoal();
  }
}
