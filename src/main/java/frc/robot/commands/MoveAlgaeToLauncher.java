// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.indexer.IndexerMoveToBeamBreak;
import frc.robot.commands.indexer.IndexerSetVelocityManualCmd;
import frc.robot.commands.launcher.LauncherSetVelocityPIDCmd;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.launcher.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveAlgaeToLauncher extends ParallelCommandGroup {
  /** Creates a new MoveAlgaeToLauncher. */
  public MoveAlgaeToLauncher(
      Launcher launcher,
      Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new IndexerMoveToBeamBreak(indexer, () -> IndexerConstants.kDirectionConstant),
        new LauncherSetVelocityPIDCmd(launcher, LauncherConstants.kVelocityFront, LauncherConstants.kVelocityBack));
  }
}
