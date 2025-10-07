package frc.robot.subsystems;

/** Interface for the pivot subsystem hardware. */
public interface PivotIO {
    /** Simple container describing the pivot state. */
    public static class PivotIOInputs {
        public double positionRads;
        public double velocityRadsPerSec;
        public double accelerationRadsPerSecSq;
        public double appliedVolts;
        public double statorCurrentAmps;
    }

    default void updateInputs(PivotIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void stop() {}

    default void resetPosition(double positionRads) {}
}
