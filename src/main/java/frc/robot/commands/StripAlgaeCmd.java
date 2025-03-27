// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmSetPositionPIDCmd;
import frc.robot.commands.intake.IntakeSetVelocityManualCmd;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StripAlgaeCmd extends SequentialCommandGroup {
  /**
   * Creates a new StripAlgae. This command assumes that the arm is already in the
   * desired L2 or L3 position.
   */
  public StripAlgaeCmd(
      IntakeSubsystem intakeSub,
      ArmSubsystem armSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new IntakeSetVelocityManualCmd(intakeSub, () -> -1).until(() -> armSub.getAlgaeOnArm()).withTimeout(5),
        new ArmSetPositionPIDCmd(armSub, () -> ArmConstants.kAngleL3));
  }
}
