package frc.robot.subsystems.drivetrain;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleSim implements SwerveModule {
  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private final PIDController drivePID;
  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController turnController;

  private double m_chassisAngularOffset = 0;

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
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(moduleSimulation.getSteerAbsoluteFacing());

    driveMotor.requestVoltage(
        Units.Volts.of(driveFeedforward.calculate(
            correctedDesiredState.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2)))
            .plus(
                Units.Volts.of(drivePID.calculate(moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond),
                    correctedDesiredState.speedMetersPerSecond / (ModuleConstants.kWheelDiameterMeters / 2)))));
    turnMotor.requestVoltage(Units.Volts.of(
        turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians(),
            correctedDesiredState.angle.getRadians())));
  }
}