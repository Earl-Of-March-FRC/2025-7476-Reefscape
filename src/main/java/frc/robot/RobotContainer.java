package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.shooter.SetShooterSpeed;
import frc.robot.commands.shooter.ShooterPID;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
        final ShooterSubsystem shooter = new ShooterSubsystem();
        private final CommandXboxController controller = new CommandXboxController(0);

        // Use WPILib's SendableChooser to allow selection of autonomous routine
        private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        public RobotContainer() {
                // Put default values for shooter speed on SmartDashboard
                SmartDashboard.putNumber("ShooterGoalSpeed", 0.5); // Default speed value (can be adjusted)
                System.out.println("SmartDashboard ShooterGoalSpeed initialized to 0.5");

                configureAutos();
                configureBindings(); // Register button bindings for shooter commands
        }

        private void configureBindings() {
                // Map button B to set a fixed shooter speed (e.g., 0.5)
                controller.b().whileTrue(new ShooterPID(shooter, () -> 60.0));
        }

        private void configureAutos() {
                // Add autonomous options to the chooser
                // autoChooser.setDefaultOption("Do Nothing", new InstantCommand()); // Default
                // option
                // autoChooser.addOption("My Custom Auto Command", new InstantCommand()); // Add
                // your custom autonomous
                // commands here

                // Update the SmartDashboard with the options
                SmartDashboard.putData("Auto Routine", autoChooser);
                System.out.println("Auto Chooser initialized with options");
        }

        public Command getAutonomousCommand() {
                // Get the selected autonomous command from the chooser
                Command selectedCommand = autoChooser.getSelected();
                System.out.println("Selected Auto Command: " + selectedCommand.getName());
                return selectedCommand;
        }

        // Optional: Add a periodic method to update SmartDashboard values dynamically
        public void updateSmartDashboard() {
                double currentSpeed = shooter.getShooterVelocity(); // Assuming you have a method to get the current
                                                                    // shooter speed
                SmartDashboard.putNumber("CurrentShooterSpeed", currentSpeed);
                System.out.println("Current Shooter Speed: " + currentSpeed);
        }
}