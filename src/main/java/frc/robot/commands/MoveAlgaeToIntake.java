// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.indexer.IndexerSetVelocityManualCmd;
import frc.robot.commands.intake.IntakeSetVelocityManualCmd;
import frc.robot.commands.launcher.LauncherSetVelocityManualCmd;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.Launcher;

/**
 * This command will move an existing algae from inside the indexer or launcher
 * to the intake for scoring in the processor. See comments in the code to view
 * the timeline of tasks being ran.
 */
public class MoveAlgaeToIntake extends SequentialCommandGroup {
  /**
   * Create a MoveAlgaeToIntake command
   * 
   * @param arm              The arm subsystem
   * @param launcher         The launcher subsystem
   * @param intake           The intake subsystem
   * @param indexer          The indexer subsystem
   * @param armMinSetpoint   The minimum arm setpoint (in radians). This will be
   *                         used to determine if the arm is in the way of the
   *                         algae.
   * @param launcherVelocity The velocity (percent) to run the launcher
   *                         motors at
   * @param intakeVelocity   The velocity (percent) to run the intake motors
   *                         at
   * @param indexerVelocity  The velocity (percent) to run the indexer
   *                         motors
   *                         at
   */
  public MoveAlgaeToIntake(
      ArmSubsystem arm,
      Launcher launcher,
      IntakeSubsystem intake,
      Indexer indexer,
      double armMinSetpoint,
      DoubleSupplier launcherVelocity,
      DoubleSupplier intakeVelocity,
      DoubleSupplier indexerVelocity) {

    addCommands(

        /*
         * Run the indexer and the launcher towards the intake until the algae reaches
         * the intake sensor or this command is interrupted
         */
        new IndexerSetVelocityManualCmd(indexer, indexerVelocity)
            .raceWith(new LauncherSetVelocityManualCmd(launcher, launcherVelocity))
            .until(() -> indexer.getIntakeSensor()),

        /*
         * Wait for the arm to move out of the way (this step is completely ignored if
         * the arm is already raised)
         */
        new WaitUntilCommand(() -> arm.getPosition() <= armMinSetpoint),

        /*
         * Start the intake and keep it going until this command is interrupted.
         * 
         * Keep running the indexer until the intake sensor no longer detects an
         * algae, or this command is interrupted.
         */
        new IntakeSetVelocityManualCmd(intake, intakeVelocity).alongWith(
            new IndexerSetVelocityManualCmd(indexer, indexerVelocity).until(() -> !indexer.getIntakeSensor())));
  }
}
