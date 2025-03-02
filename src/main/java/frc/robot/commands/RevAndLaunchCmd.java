// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.IndexerSetVelocityManualCmd;
import frc.robot.commands.launcher.LauncherSetVelocityPIDCmd;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.launcher.Launcher;

/**
 * This command will start the launcher and wait for its velocity to meet the
 * set tolerance. The indexer will work to move the algae towards the launcher,
 * and feed it in once the launcher is fully revved up.
 */
public class RevAndLaunchCmd extends SequentialCommandGroup {
  /**
   * Create a MoveAlgaeToLauncher command
   * 
   * @param launcher                       The launcher subsystem
   * @param indexer                        The indexer subsystem
   * @param launcherFrontReferenceVelocity The velocity (PID) of the front rollers
   *                                       to launch the algae
   * @param launcherBackReferenceVelocity  The velocity (PID) of the back rollers
   *                                       to launch the algae
   * @param launcherFrontTolerance         The PID tolerance (positive) of the
   *                                       launcher's front roller
   * @param launcherBackTolerance          The PID tolerance (positive) of the
   *                                       launcher's back roller
   * @param indexerVelocity                The velocity (percent) to move algae
   *                                       towards the launcher
   * @param releaseSignal                  Returns true when the driver releases
   *                                       the trigger. If the launcher's velocity
   *                                       doesn't meet the tolerance, the command
   *                                       will end without launching the
   *                                       algae.
   * @param launchTimeout                  Timeout (seconds) after the driver
   *                                       triggers a launch.
   */
  public RevAndLaunchCmd(
      Launcher launcher,
      Indexer indexer,
      DoubleSupplier launcherFrontReferenceVelocity,
      DoubleSupplier launcherBackReferenceVelocity,
      double launcherFrontTolerance,
      double launcherBackTolerance,
      DoubleSupplier indexerVelocity,
      BooleanSupplier releaseSignal,
      double launchTimeout) {

    /*
     * Supplier returns true when the launcher rollers' recorded velocity is within
     * the range of their tolerances.
     */
    BooleanSupplier launcherIsRevved = () -> {
      return MathUtil.isNear(launcherFrontReferenceVelocity.getAsDouble(), launcher.getFrontVelocity(),
          launcherFrontTolerance)
          && MathUtil.isNear(launcherBackReferenceVelocity.getAsDouble(), launcher.getBackVelocity(),
              launcherBackTolerance);
    };

    addCommands(
        /*
         * Move the algae towards the launcher while the launcher is revving up. This
         * ends when the driver releases the trigger.
         */
        new ParallelCommandGroup(
            /* Move algae to launcher sensor */
            new IndexerSetVelocityManualCmd(indexer, indexerVelocity).until(() -> indexer.getLauncherSensor()),

            /* Run launcher in order to rev it up */
            new LauncherSetVelocityPIDCmd(launcher, launcherFrontReferenceVelocity.getAsDouble(),
                launcherBackReferenceVelocity.getAsDouble())

        ).until(() -> releaseSignal.getAsBoolean()),

        /*
         * Launch the algae if the launcher is already revved. Otherwise, this step will
         * be ignored and the command will end.
         * 
         * Once the above condition passes, this step will end once none of the sensors
         * detect the algae, meaning it was launched. We are checking both sensors
         * because there is a chance the driver tries to launch before the algae makes
         * it all the way to the launcher sensor.
         * 
         * This step also times out in case for some reason, the algae isn't being
         * launched.
         */
        new ParallelCommandGroup(
            /* Launch the algae. */
            new IndexerSetVelocityManualCmd(indexer, indexerVelocity),
            new LauncherSetVelocityPIDCmd(launcher, launcherFrontReferenceVelocity.getAsDouble(),
                launcherBackReferenceVelocity.getAsDouble())

        ).onlyIf(launcherIsRevved).until(() -> !indexer.getBothSensors()).withTimeout(launchTimeout));
  }
}
