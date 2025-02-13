// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerSensor;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * Command that runs the indexer in a certain direction using closed-loop
 * velocity.
 */
public class IndexToSubsystemCmd extends Command {
  private final IndexerSubsystem indexerSub;
  private final DoubleSupplier indexVelocity;

  /**
   * Runs the indexer towards a subsystem.
   * 
   * @param indexVelocity The velocity to run the indexer. Will determine the
   *                      direction to run at, therefore which subsystem to run
   *                      to.
   * 
   * @return A command requiring the indexer.
   */
  public IndexToSubsystemCmd(IndexerSubsystem indexerSub, DoubleSupplier indexVelocity) {
    this.indexerSub = indexerSub;
    this.indexVelocity = indexVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean sensorIsTripped;
    // Check if the velocity runs the indexer towards the launcher
    if (Math.signum(indexVelocity.getAsDouble()) == Math.signum(IndexerConstants.kDirectionConstant)) {
      // Move towards the launcher
      sensorIsTripped = indexerSub.getLauncherSensor();
    } else {
      // Move towards the intake
      sensorIsTripped = indexerSub.getIntakeSensor();
    }
    // If sensor is tripped, stop the indexer. Else, continue to set it to the
    // velocity.
    indexerSub.setReferenceVelocity(sensorIsTripped ? 0 : indexVelocity.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSub.setReferenceVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
