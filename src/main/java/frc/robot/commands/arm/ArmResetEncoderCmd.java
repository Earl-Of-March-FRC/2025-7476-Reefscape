// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmResetEncoderCmd extends InstantCommand {
  ArmSubsystem armSub;

  public ArmResetEncoderCmd(ArmSubsystem armSub) {
    this.armSub = armSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSub.resetPosition();
  }
}
