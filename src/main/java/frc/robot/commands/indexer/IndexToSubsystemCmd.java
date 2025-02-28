// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.indexer.Indexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexToSubsystemCmd extends Command {
  private final Indexer indexerSub;
  private final DoubleSupplier percentVelocity;

  /**
   * Creates a new IndexToSubsystemCmd.
   * Runs the indexer towards a subsystem.
   * 
   * @param indexerSub      The indexer subsystem used by this command.
   * @param percentVelocity The velocity to run the indexer. Will determine the
   *                        direction to run at, therefore which subsystem to run
   *                        to.
   */
  public IndexToSubsystemCmd(Indexer indexerSub, DoubleSupplier percentVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexerSub = indexerSub;
    this.percentVelocity = percentVelocity;
    addRequirements(indexerSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexerSub.setVelocity(percentVelocity.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSub.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.signum(percentVelocity.getAsDouble()) != Math.signum(IndexerConstants.kDirectionConstant)) {
      // Move towards the launcher
      if (!indexerSub.getLauncherSensor()) {
        return true;
      }
    } else {
      // Move towards the intake
      if (!indexerSub.getIntakeSensor()) {
        return true;
      }
    }
    return false;
  }
}
