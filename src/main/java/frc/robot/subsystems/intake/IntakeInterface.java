package frc.robot.subsystems.intake;

public interface IntakeInterface {

    public void setRollerSpeed(double speed);

    public void setPivotSpeed(double speed);

    public void pivotHold(double currentAngle);

    public double getPivotPosition();

    public double getPivotRate();

}
