package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {

  public SwerveModuleState getState();

  public SwerveModulePosition getPosition();

  public void setDesiredState(SwerveModuleState desiredState);

  public default void resetEncoders() {

  }

}
