// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShoulderMoveAuto extends Command {

  private IntakeSubsystem intakeSub;
  private ProfiledPIDController pidController;

  /** Creates a new ShoulderMoveAuto. */
  public ShoulderMoveAuto(IntakeSubsystem intakeSub, double goalAngle) {

    this.intakeSub = intakeSub;

    // creates Profiled PID Controller
    pidController = new ProfiledPIDController(
        // sets PID gains
        IntakeConstants.kPShoulderController,
        IntakeConstants.kIShoulderController,
        IntakeConstants.kDShoulderController,

        // sets constraints
        IntakeConstants.kShoulderControllerConstraints);

    // sets goal and tolerance for controller
    pidController.setGoal(goalAngle);
    pidController.setTolerance(1);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset(intakeSub.getShoulderPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.setShoulderSpeed(pidController.calculate(intakeSub.getShoulderPosition()));
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
