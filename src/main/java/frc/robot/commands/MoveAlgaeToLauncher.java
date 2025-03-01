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
import frc.robot.subsystems.arm.ArmSubsystem;
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
   * @param launcher                       The launcher subsystem
   * @param intake                         The intake subsystem
   * @param indexer                        The indexer subsystem
   * @param launcherFrontReferenceVelocity The velocity (PID) of the front rollers
   *                                       to
   *                                       launch the algae at
   * @param launcherBackReferenceVelocity  The velocity (PID) of the back rollers
   *                                       to
   *                                       launch the algae at
   * @param launcherFrontTolerance         The PID tolerance of the launcher's
   *                                       front
   *                                       roller
   * @param launcherBackTolerance          The PID tolerance of the launcher's
   *                                       back roller
   * @param intakeVelocity                 The velocity (percent) to intake algae
   * @param indexerVelocity                The velocity (percent) to move algae
   *                                       from the
   *                                       intake to the launcher
   * @param releaseSignal                  Returns true when the driver releases
   *                                       the trigger.
   *                                       If the launcher's velocity doesn't meet
   *                                       the tolerance, the command will be
   *                                       cancelled without
   *                                       launching the algae.
   * @param launchTimeout                  Timeout (seconds) after the driver
   *                                       triggers a launch.
   */
  public MoveAlgaeToLauncher(
      ArmSubsystem arm,
      DoubleSupplier armIntakePosition,
      double armTolerance,
      Launcher launcher,
      IntakeSubsystem intake,
      Indexer indexer,
      double launcherFrontReferenceVelocity,
      double launcherBackReferenceVelocity,
      double launcherFrontTolerance,
      double launcherBackTolerance,
      DoubleSupplier intakeVelocity,
      DoubleSupplier indexerVelocity,
      BooleanSupplier releaseSignal,
      double launchTimeout) {

    /*
     * Supplier returns true when the launcher rollers' recorded velocity is within
     * the range of the tolerance.
     */
    BooleanSupplier launcherIsRevved = () -> {
      // Get absolute values
      final double absFront = Math.abs(launcher.getFrontVelocity()); // Front velocity
      final double absBack = Math.abs(launcher.getBackVelocity()); // Back velocity
      final double absFrontSetpoint = Math.abs(launcherFrontReferenceVelocity); // Front setpoint
      final double absBackSetpoint = Math.abs(launcherBackReferenceVelocity); // Back setpoint

      // Condition: (setpoint - tolerance) <= velocity <= (setpoint + tolerance)
      return

      // Front roller
      (absFront >= absFrontSetpoint - launcherFrontTolerance
          && absFront <= absFrontSetpoint + launcherFrontTolerance)

          // Back roller
          && (absBack >= absBackSetpoint - launcherBackTolerance
              && absBack <= absBackSetpoint + launcherBackTolerance);
    };

    /*
     * Supplier returns true when the arm position is within the tolerance.
     */
    BooleanSupplier armIntaking = () -> {
      // Get absolute values
      final double absPos = Math.abs(arm.getPosition()); // Front velocity
      final double absSetpoint = Math.abs(armIntakePosition.getAsDouble()); // Back setpoint

      // Condition: (setpoint - tolerance) <= velocity <= (setpoint + tolerance)
      return

      // Position
      (absPos >= absSetpoint - armTolerance
          && absPos <= absSetpoint + armTolerance);
    };

    addCommands(
        /*
         * Move the algae towards the launcher while the launcher is revving up. This
         * ends when the launcher is fully revved, or the driver releases the trigger.
         */
        new ParallelCommandGroup(
            /* If arm is in intaking position, run the intake. */
            new IntakeSetVelocityManualCmd(intake, intakeVelocity).onlyWhile(armIntaking)
                .until(() -> indexer.getLauncherSensor()),
            /* Move algae to launcher sensor */
            new IndexerSetVelocityManualCmd(indexer, indexerVelocity).until(() -> indexer.getLauncherSensor()),

            /* Run launcher in order to rev it up */
            new LauncherSetVelocityPIDCmd(launcher, launcherFrontReferenceVelocity, launcherBackReferenceVelocity)

        ).until(() -> launcherIsRevved.getAsBoolean() || releaseSignal.getAsBoolean()),

        /*
         * Launch the algae if the launcher is already revved. Otherwise, this step will
         * be ignored and the command will end.
         * 
         * Once the above condition passes, this step will end once the algae leaves the
         * launcher sensor, which indicates that it was launched.
         * 
         * This step also times out in case for some reason, the algae isn't being
         * launched.
         */
        new ParallelCommandGroup(
            /*
             * If the algae is still not at the launcher sensor (for some reason), the
             * intake will continue to run, in case the algae is still in the intake. This
             * will end once the algae reaches the launcher sensor.
             */
            new IntakeSetVelocityManualCmd(intake, intakeVelocity).onlyIf(() -> !indexer.getLauncherSensor())
                .until(() -> indexer.getLauncherSensor()),

            /* Launch the algae. */
            new IndexerSetVelocityManualCmd(indexer, indexerVelocity),
            new LauncherSetVelocityPIDCmd(launcher, launcherFrontReferenceVelocity, launcherBackReferenceVelocity)

        ).onlyIf(launcherIsRevved).until(() -> !indexer.getLauncherSensor()).withTimeout(launchTimeout));
  }
}
