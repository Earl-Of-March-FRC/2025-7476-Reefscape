package frc.robot.subsystems.drivetrain;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleSim implements SwerveModule {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private final PIDController drivePID;
  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController turnController;

  public SwerveModuleSim(SwerveModuleSimulation moduleSim) {
    moduleSimulation = moduleSim;

    drivePID = new PIDController(ModuleConstants.kDrivingPSim, ModuleConstants.kDrivingISim,
        ModuleConstants.kDrivingDSim);
    driveFeedforward = new SimpleMotorFeedforward(0.0, ModuleConstants.kDrivingFFSim);
    turnController = new PIDController(ModuleConstants.kTurningPSim, ModuleConstants.kTurningISim,
        ModuleConstants.kTurningDSim);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
        .withCurrentLimit(Units.Amps.of(50));
    turnMotor = moduleSimulation.useGenericControllerForSteer()
        .withCurrentLimit(Units.Amps.of(20));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond)
            * ModuleConstants.kWheelDiameterMeters / 2,
        moduleSimulation.getSteerAbsoluteFacing());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        moduleSimulation.getDriveWheelFinalPosition().in(Units.Radians)
            * ModuleConstants.kWheelDiameterMeters / 2,
        moduleSimulation.getSteerAbsoluteFacing());
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    driveMotor.requestVoltage(
        Units.Volts.of(driveFeedforward.calculate(
            state.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2)))
            .plus(
                Units.Volts.of(drivePID.calculate(moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond),
                    state.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2)))));
    turnMotor.requestVoltage(Units.Volts.of(
        turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians(), state.angle.getRadians())));
  }
}