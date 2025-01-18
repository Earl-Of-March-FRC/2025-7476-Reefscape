package frc.robot.subsystems.intake;

public interface IntakeInterface {

    public void setRollerSpeed(double speed);

    public void setShoulderSpeed(double speed);

    public void shoulderHold(double currentAngle);

    public double getShoulderPosition();

    public double getShoulderRate();

}
