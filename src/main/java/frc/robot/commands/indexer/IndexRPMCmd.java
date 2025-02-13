// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/**
 * Command that runs the indexer in a certain RPM using closed-loop
 * velocity.
 */
public class IndexRPMCmd extends InstantCommand {
  private final IndexerSubsystem indexerSub;
  private final DoubleSupplier velocity;

  /**
   * Runs the indexer at a specific RPM.
   * 
   * @param velocity The velocity to run at
   * @return A command requiring the indexer.
   */
  public IndexRPMCmd(IndexerSubsystem indexerSub, DoubleSupplier velocity) {
    this.indexerSub = indexerSub;
    this.velocity = velocity;
    addRequirements(indexerSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexerSub.setReferenceVelocity(velocity.getAsDouble());
  }
}
