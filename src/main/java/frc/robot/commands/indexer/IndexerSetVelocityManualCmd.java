// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerSetVelocityManualCmd extends Command {
  private final Indexer indexerSub;
  private final DoubleSupplier percentVelocity;

  /**
   * Creates a new IndexerManualVelocity.
   * Manually control the speed of the indexer
   * 
   * @param indexerSub      The indexer subsystem used by this command.
   * @param percentVelocity A joystick input between -1.0 and +1.0
   *
   */
  public IndexerSetVelocityManualCmd(Indexer indexerSub, DoubleSupplier percentVelocity) {
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
