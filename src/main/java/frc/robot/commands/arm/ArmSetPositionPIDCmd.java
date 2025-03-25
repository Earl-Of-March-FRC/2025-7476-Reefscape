// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.ArmSubsystem;

/**
 * This command uses closed-loop control to move the arm to the desired angle.
 */
public class ArmSetPositionPIDCmd extends InstantCommand {

  private ArmSubsystem armSub;
  private Supplier<Angle> referenceAngle;

  /**
   * Sets the reference value for the arm subsystem's closed-loop controller. It
   * will automatically begin moving towards the desired angle.
   * 
   * @param armSub         The instance of the ArmSubsystem class to be used.
   * @param referenceAngle The goal angle for the arm to move to, in degrees.
   */
  public ArmSetPositionPIDCmd(ArmSubsystem armSub, Supplier<Angle> referenceAngle) {

    this.armSub = armSub;
    this.referenceAngle = referenceAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  @Override
  public void initialize() {
    armSub.clearOffset();
    armSub.setReferencePosition(referenceAngle.get());
  }
}
