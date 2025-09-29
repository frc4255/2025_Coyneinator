package frc.robot.subsystems;

/** Hardware abstraction for the wrist roll subsystem. */
public interface WristRollIO {
    /** Container describing wrist roll sensor data. */
    public static class WristRollIOInputs {
        public double positionRads;
        public double velocityRadsPerSec;
        public double accelerationRadsPerSecSq;
        public double appliedVolts;
        public double statorCurrentAmps;
    }

    default void updateInputs(WristRollIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void stop() {}

    default void resetPosition(double positionRads) {}
}
