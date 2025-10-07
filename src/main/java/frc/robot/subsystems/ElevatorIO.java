package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters;
        public double velocityMetersPerSecond;
        public double accelerationMetersPerSecondSq;
        public double appliedVolts;
        public double currentAmps;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void runOpenLoop(double percent) {}

    default void stop() {}

    default void resetPosition(double positionMeters) {}
}
