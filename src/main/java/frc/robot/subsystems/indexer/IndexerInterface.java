package frc.robot.subsystems.indexer;

public interface IndexerInterface {
    /**
     * Set the output speed of the motors.
     * 
     * @param percent Percentage between -1.0 and +1.0
     */
    public void setSpeed(double percent);

    /**
     * Get the output speed of the motors.
     * 
     * @return Percent between -1.0 and +1.0
     */
    public default double getSpeed() {
        return 0D;
    }

    /**
     * Set the input voltage of the motors.
     * 
     * @param voltage Voltage between -12V and +12V, or refer to
     *                {@link #getMaxVoltage()}.
     */
    public void setVoltage(double voltage);

    /**
     * Get the input voltage of the motors.
     * 
     * @return Voltage between -12V and +12V, or refer to
     *         {@link #getMaxVoltage()}.
     */
    public default double getVoltage() {
        return 0D;
    }

    /**
     * Get the maximum voltage applicable to the motors.
     * 
     * @return Max voltage according to motor brand. Returns 12V by default.
     */
    public default double getMaxVoltage() {
        return 12D;
    };

    /**
     * Get the value of the first sensor.
     * 
     * @return {@code true} if sensor is tripped or not connected, {@code false} if
     *         not tripped.
     */
    public default boolean getSensor1() {
        return true;
    }

    /**
     * Get the value of the second sensor.
     * 
     * @return {@code true} if sensor is tripped or not connected, {@code false} if
     *         not tripped.
     */
    public default boolean getSensor2() {
        return true;
    }
}
