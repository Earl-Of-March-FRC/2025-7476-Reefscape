// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.indexer.IndexerMoveToBeamBreak;
import frc.robot.commands.indexer.IndexerSetVelocityManualCmd;
import frc.robot.commands.intake.IntakeSetVelocityManualCmd;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveAlgaeToIntake extends SequentialCommandGroup {
  /** Creates a new MoveAlgaeToLauncher. */
  public MoveAlgaeToIntake(
      ArmSubsystem arm,
      IntakeSubsystem intake,
      Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new IndexerMoveToBeamBreak(indexer, () -> -IndexerConstants.kDirectionConstant),
        new WaitUntilCommand(() -> arm.getPosition() >= -Math.PI * 0.75),
        new IntakeSetVelocityManualCmd(intake, () -> -1)
            .alongWith(new IndexerSetVelocityManualCmd(indexer, () -> intake.getVelocity())));
  }
}
