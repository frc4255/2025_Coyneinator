package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface WristPitchIO {
    @AutoLog
    public static class WristPitchIOInputs {
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double accelerationRadiansPerSecondSq;
        public double appliedVolts;
        public double currentAmps;
    }

    default void updateInputs(WristPitchIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void runOpenLoop(double percent) {}

    default void stop() {}

    default void resetPosition(double radians) {}
}
