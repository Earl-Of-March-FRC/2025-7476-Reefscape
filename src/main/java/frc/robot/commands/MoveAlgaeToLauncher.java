// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.LauncherConstants;
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
public class MoveAlgaeToLauncher extends ParallelCommandGroup {
  /**
   * Create a MoveAlgaeToLauncher command
   * 
   * @param launcher               The launcher subsystem
   * @param intake                 The intake subsystem
   * @param indexer                The indexer subsystem
   * @param launcherFrontVelocity  The velocity (PID) of the front rollers to
   *                               launch the algae at
   * @param launcherBackVelocity   The velocity (PID) of the back rollers to
   *                               launch the algae at
   * @param launcherFrontTolerance The PID tolerance of the launcher's front
   *                               roller
   * @param launcherBackTolerance  The PID tolerance of the launcher's back roller
   * @param intakeVelocity         The velocity (percent) to intake algae
   * @param indexerVelocity        The velocity (percent) to move algae from the
   *                               intake to the launcher
   * @param launchSignal           Returns true when the driver chooses to launch.
   *                               If the launcher's velocity doesn't meet the
   *                               tolerance, the command will be cancelled
   *                               without
   *                               launching the algae.
   * @param launchTimeout          Timeout (seconds) after the driver triggers a
   *                               launch.
   */
  public MoveAlgaeToLauncher(
      Launcher launcher,
      IntakeSubsystem intake,
      Indexer indexer,
      double launcherFrontVelocity,
      double launcherBackVelocity,
      double launcherFrontTolerance,
      double launcherBackTolerance,
      DoubleSupplier intakeVelocity,
      DoubleSupplier indexerVelocity,
      BooleanSupplier launchSignal,
      double launchTimeout) {

    /*
     * Supplier returns true when the launcher rollers' recorded velocity is within
     * the range of the tolerance.
     */
    BooleanSupplier launcherIsRevved = () -> {
      // Get absolute values
      final double absFront = Math.abs(launcher.getFrontVelocity()); // Front velocity
      final double absBack = Math.abs(launcher.getBackVelocity()); // Back velocity
      final double absFrontSetpoint = Math.abs(launcherFrontVelocity); // Front setpoint
      final double absBackSetpoint = Math.abs(launcherBackVelocity); // Back setpoint

      // Condition: (setpoint - tolerance) <= velocity <= (setpoint + tolerance)
      return

      // Front roller
      (absFront >= absFrontSetpoint - launcherFrontTolerance
          && absFront <= absFrontSetpoint + launcherFrontTolerance)

          // Back roller
          && (absBack >= absBackSetpoint - launcherBackTolerance
              && absBack <= absBackSetpoint + launcherBackTolerance);
    };

    /* These commands run in unison */
    addCommands(
        /*
         * Start revving up the launcher while the algae is being intaked/indexed by the
         * other subsystems
         */
        new LauncherSetVelocityPIDCmd(launcher, launcherFrontVelocity, launcherBackVelocity)
            /*
             * Move on from revving when the algae reaches the launcher sensor and the
             * launcher has been fully revved up. This is mainly to change the .until
             * conditions, and assumes that the indexer is actively moving the algae into
             * the launcher.
             */
            .until(() -> indexer.getLauncherSensor() && launcherIsRevved.getAsBoolean())
            /*
             * Keep running the launcher at the setpoint (to launch the algae) until the
             * algae leaves the launcher sensor, or time runs out.
             */
            .andThen(new LauncherSetVelocityPIDCmd(launcher, launcherFrontVelocity, launcherBackVelocity)
                .until(() -> !indexer.getLauncherSensor()).withTimeout(launchTimeout)),

        /* Will run the following commands one at a time instead of in unison. */
        new SequentialCommandGroup(
            /*
             * Move the algae from the intake (or from anywhere else in the robot) to the
             * launcher sensor.
             */
            new IntakeSetVelocityManualCmd(intake, intakeVelocity).alongWith(
                new IndexerSetVelocityManualCmd(indexer, indexerVelocity)).until(() -> indexer.getLauncherSensor()),
            /* Wait for the driver to signal a launch. */
            new WaitUntilCommand(launchSignal),
            /*
             * If the launcher has been revved, the indexer will start again to move the
             * algae from the launcher sensor into the launcher. This will stop
             * when the algae leaves the launcher sensor, or time runs out.
             */
            new IndexerSetVelocityManualCmd(indexer, indexerVelocity)
                .onlyIf(launcherIsRevved))
            .until(() -> !indexer.getLauncherSensor())
            .withTimeout(launchTimeout)

    );
  }
}
