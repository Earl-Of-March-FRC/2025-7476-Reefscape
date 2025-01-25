package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.VisionSubsystem;

public class SetPipelineCommand extends Command {
  private final VisionSubsystem visionSubsystem;
  private final CommandXboxController xboxController;
  private boolean wasButtonPressedLast = false;

  // Constructor takes the VisionSubsystem and CommandXboxController
  public SetPipelineCommand(VisionSubsystem visionSubsystem, CommandXboxController xboxController) {
    this.visionSubsystem = visionSubsystem;
    this.xboxController = xboxController;
    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    // On initialization, set the pipeline to 0 (default)
    visionSubsystem.setPipeline(0);
  }

  @Override
  public void execute() {
    // Check if the Y button is being pressed
    boolean isButtonPressed = xboxController.y().getAsBoolean();

    // Only change the pipeline when the button is released (button goes from
    // pressed to not pressed)
    if (isButtonPressed && !wasButtonPressedLast) {
      // Switch pipeline to 1 when the button is first pressed
      visionSubsystem.setPipeline(1);
      System.out.println("Algae");
    } else if (!isButtonPressed && wasButtonPressedLast) {
      // Switch pipeline back to 0 when the button is released
      visionSubsystem.setPipeline(0);
      System.out.println("April Tag");
    }

    // Update the button press state for the next cycle
    wasButtonPressedLast = isButtonPressed;
  }

  @Override
  public boolean isFinished() {
    // This command will continue running indefinitely until interrupted
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Ensure the pipeline is reset to 0 when the command ends
    visionSubsystem.setPipeline(0);
  }
}
