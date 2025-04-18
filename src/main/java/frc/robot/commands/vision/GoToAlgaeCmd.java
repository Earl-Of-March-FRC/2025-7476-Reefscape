package frc.robot.commands.vision;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.AlgaeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class GoToAlgaeCmd extends Command {
  private final AlgaeSubsystem algaeSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private Command pathCommand;
  private final Timer timer = new Timer(); // Timer to track elapsed time

  /** Creates a new GoToAlgaeCmd. */
  public GoToAlgaeCmd(AlgaeSubsystem algaeSubsystem, IntakeSubsystem intakeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.addRequirements(algaeSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {
    timer.restart(); // Stop, reset, and start timer

    // Switch to Algae pipeline
    algaeSubsystem.setPipeline(Constants.Vision.PhotonConstants.kAlgaePipeline);
  }

  @Override
  public void execute() {

    // // If the path has changed, cancel the old path command and start a new one
    // if (pathCommand != null) {
    // pathCommand.cancel();
    // }
    // pathCommand = AutoBuilder.followPath(path);
    // pathCommand.schedule();

    // Start the intake
    intakeSubsystem.setVelocity(IntakeConstants.kDefaultAlgaeIntake);

    // Update the path every second
    if (timer.hasElapsed(1.0)) {
      // Cancel the old path command
      if (pathCommand != null) {
        pathCommand.cancel();
      }

      // algaeSubsystem.updateTargetPose(); // Update the target pose
      // If (and when) the path finishes, gracefully end GoToAlgaeCmd
      pathCommand = AutoBuilder.followPath(algaeSubsystem.getPath()).andThen(
          () -> end(false));
      pathCommand.schedule();

      // Reset the timer for the next update
      timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Ensure the path command and intake stop when command ends
    if (pathCommand != null) {
      pathCommand.cancel();
    }
    intakeSubsystem.stopIntake();

    // Switch back to the AprilTag pipeline
    algaeSubsystem.setPipeline(Constants.Vision.PhotonConstants.kAprilTagPipeline);
  }

  @Override
  public boolean isFinished() {
    // Keep running as long as the button is held down
    return false;
  }
}
