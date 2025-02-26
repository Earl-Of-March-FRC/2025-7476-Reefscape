package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.AlgaeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
    timer.reset(); // Reset timer when the command starts
    timer.start(); // Start the timer
  }

  @Override
  public void execute() {
    // The button state is automatically handled by whileHeld, no need for manual
    // checking

    // If the button was pressed, start tracking and moving
    algaeSubsystem.updateTargetPose();
    PathPlannerPath path = algaeSubsystem.getPath();

    // If the path has changed, cancel the old path command and start a new one
    if (pathCommand != null) {
      pathCommand.cancel();
    }
    pathCommand = AutoBuilder.followPath(path);
    pathCommand.schedule();

    // Start the intake
    intakeSubsystem.setVelocity(IntakeConstants.kDefaultAlgaeIntake);

    // Update the path every 1 second
    if (timer.hasElapsed(1.0)) {
      algaeSubsystem.updateTargetPose(); // Update the target pose
      PathPlannerPath newPath = algaeSubsystem.getPath();

      // Cancel the old path command and schedule the new one
      if (pathCommand != null) {
        pathCommand.cancel();
      }
      pathCommand = AutoBuilder.followPath(newPath);
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
  }

  @Override
  public boolean isFinished() {
    // Keep running as long as the button is held down
    return false;
  }
}
