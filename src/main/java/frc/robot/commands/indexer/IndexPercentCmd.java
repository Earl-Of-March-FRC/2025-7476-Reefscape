// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

/*
 * Open-loop control of the indexer.
 */
public class IndexPercentCmd extends Command {
  private final IndexerSubsystem indexerSub;
  private final DoubleSupplier speed;

  /**
   * Manually control the speed percentage of the indexer
   * 
   * @param input A joystick input between -1.0 and +1.0
   * @return A command requiring the indexer.
   */
  public IndexPercentCmd(IndexerSubsystem indexerSub, DoubleSupplier speed) {
    this.indexerSub = indexerSub;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexerSub.setSpeed(speed.getAsDouble());
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
