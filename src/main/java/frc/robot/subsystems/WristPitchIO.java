package frc.robot.subsystems;

/** Interface describing the hardware layer for the wrist pitch subsystem. */
public interface WristPitchIO {
    /** Container describing current wrist pitch sensor readings. */
    public static class WristPitchIOInputs {
        public double positionRads;
        public double velocityRadsPerSec;
        public double accelerationRadsPerSecSq;
        public double appliedVolts;
        public double statorCurrentAmps;
    }

    default void updateInputs(WristPitchIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void stop() {}

    default void resetPosition(double positionRads) {}
}
