package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.SetPipelineCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final VisionSubsystem visionSubsystem;
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Initialize VisionSubsystem
    this.visionSubsystem = new VisionSubsystem();

    // Create and bind SetPipelineCommand to the Y button press
    SetPipelineCommand setPipelineCommand = new SetPipelineCommand(visionSubsystem, driverController);
    driverController.y().onTrue(setPipelineCommand);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Any additional trigger or button mappings can be added here
  }

  public Command getAutonomousCommand() {
    return null; // Autonomous command goes here
  }
}
