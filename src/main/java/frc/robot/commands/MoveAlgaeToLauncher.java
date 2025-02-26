// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.IndexerSetVelocityManualCmd;
import frc.robot.commands.intake.IntakeSetVelocityManualCmd;
import frc.robot.commands.launcher.LauncherSetVelocityPIDCmd;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.Launcher;

/**
 * This command will move an existing algae from inside the indexer or intake to
 * the launcher for scoring in the barge. See comments in the code to view
 * the timeline of tasks being ran.
 */
public class MoveAlgaeToLauncher extends SequentialCommandGroup {

  /**
   * Create a MoveAlgaeToLauncher command
   * 
   * @param launcher              The launcher subsystem
   * @param intake                The intake subsystem
   * @param indexer               The indexer subsystem
   * @param launcherFrontVelocity The velocity (PID) to run the launcher's front
   *                              rollers at
   * @param launcherBackVelocity  The velocity (PID) to run the launcher's back
   *                              rollers at
   * @param intakeVelocity        The velocity (percent) to run the intake at
   * @param indexerVelocity       The velocity (percent) to run the indexer at
   */
  public MoveAlgaeToLauncher(
      Launcher launcher,
      IntakeSubsystem intake,
      Indexer indexer,
      double launcherFrontVelocity,
      double launcherBackVelocity,
      DoubleSupplier intakeVelocity,
      DoubleSupplier indexerVelocity) {

    addCommands(
        /*
         * Run the intake and the indexer towards the launcher until the algae makes it
         * to the launcher sensor, or this command is interrupted.
         */
        new IntakeSetVelocityManualCmd(intake, intakeVelocity)
            .alongWith(new IndexerSetVelocityManualCmd(indexer, indexerVelocity))
            .until(() -> indexer.getLauncherSensor()),

        /*
         * Start the launcher and keep it going until this command is interrupted.
         * 
         * Keep running the indexer until the launcher sensor no longer detects an
         * algae, or this command is interrupted.
         */
        new LauncherSetVelocityPIDCmd(launcher, launcherFrontVelocity, launcherBackVelocity).alongWith(
            new IndexerSetVelocityManualCmd(indexer, indexerVelocity).until(() -> !indexer.getLauncherSensor())));
  }
}
