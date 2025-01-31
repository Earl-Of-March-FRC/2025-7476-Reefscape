package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class SetPipelineCommand extends Command {
  private final VisionSubsystem limelightSubsystem;
  private final Trigger buttonTrigger;
  private boolean wasButtonPressedLast = false;

  // Constructor takes the VisionSubsystem and Trigger (button)
  public SetPipelineCommand(VisionSubsystem visionSubsystem, Trigger buttonTrigger) {
    this.limelightSubsystem = visionSubsystem;
    this.buttonTrigger = buttonTrigger;
    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    // On initialization, set the pipeline to 0 (default)
    limelightSubsystem.setPipeline(0);
  }

  @Override
  public void execute() {
    // Check if the button is being pressed
    boolean isButtonPressed = buttonTrigger.getAsBoolean();

    // Only change the pipeline when the button is released (button goes from
    // pressed to not pressed)
    if (isButtonPressed && !wasButtonPressedLast) {
      // Switch pipeline to 1 when the button is first pressed
      limelightSubsystem.setPipeline(1);
      Logger.recordOutput("Limelight/Pipeline", "Switched to pipeline 1 (Algae)"); // Log pipeline switch
    } else if (!isButtonPressed && wasButtonPressedLast) {
      // Switch pipeline back to 0 when the button is released
      limelightSubsystem.setPipeline(0);
      Logger.recordOutput("Limelight/Pipeline", "Switched to pipeline 0 (April Tag)"); // Log pipeline switch
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
    limelightSubsystem.setPipeline(0);
    Logger.recordOutput("Limelight/Pipeline", "Pipeline reset to 0"); // Log reset action

  }
}