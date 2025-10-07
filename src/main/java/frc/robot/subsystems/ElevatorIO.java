package frc.robot.subsystems;

/**
 * Hardware interface for the elevator subsystem. Implementations provide the
 * direct communication with hardware (real or simulated) while the
 * {@link Elevator} class focuses on control logic.
 */
public interface ElevatorIO {
    /** Container for elevator sensor inputs. */
    public static class ElevatorIOInputs {
        /** Elevator position in meters. */
        public double positionMeters;
        /** Elevator velocity in meters per second. */
        public double velocityMetersPerSecond;
        /** Elevator acceleration in meters per second squared. */
        public double accelerationMetersPerSecondSq;
        /** Applied motor voltage. */
        public double appliedVolts;
        /** Estimated stator current draw in amps. */
        public double statorCurrentAmps;
    }

    /** Refreshes the latest sensor inputs from the implementation. */
    default void updateInputs(ElevatorIOInputs inputs) {}

    /** Applies the requested voltage to the elevator motors. */
    default void setVoltage(double volts) {}

    /** Stops any motion on the elevator. */
    default void stop() {}

    /** Resets the internal sensor position to the provided value. */
    default void resetPosition(double positionMeters) {}
}
