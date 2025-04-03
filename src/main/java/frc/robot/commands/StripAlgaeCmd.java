// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmSetPositionPIDCmd;
import frc.robot.commands.indexer.IndexerSetVelocityManualCmd;
import frc.robot.commands.intake.IntakeSetVelocityManualCmd;
import frc.robot.commands.launcher.LauncherSetVelocityManualCmd;
import frc.robot.commands.launcher.LauncherSetVelocityPIDCmd;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StripAlgaeCmd extends SequentialCommandGroup {
  /**
   * Creates a new StripAlgae. This command assumes that the arm is already in the
   * desired L2 or L3 position. This will strip, and attempt to intake alge.
   */
  public StripAlgaeCmd(
      IntakeSubsystem intake,
      ArmSubsystem arm,
      Launcher launcher,
      Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Strip and guide algae into funnel
        new IntakeSetVelocityManualCmd(intake, () -> -1).until(() -> arm.getAlgaeOnArm()).withTimeout(5),
        new ArmSetPositionPIDCmd(arm, () -> ArmConstants.kAngleL3),
        new WaitUntilCommand(() -> indexer.getLauncherSensor()).withTimeout(3),

        // Intake from launcher
        new ParallelCommandGroup(
            new LauncherSetVelocityPIDCmd(launcher, () -> -launcher.getPreferredFrontVelocity(),
                () -> -launcher.getPreferredBackVelocity()),
            new IndexerSetVelocityManualCmd(indexer, () -> -1)

        ).until(() -> !indexer.getLauncherSensor()));
  }
}
