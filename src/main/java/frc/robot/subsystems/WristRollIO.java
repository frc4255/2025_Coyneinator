package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface WristRollIO {
    @AutoLog
    public static class WristRollIOInputs {
        public double positionRadians;
        public double velocityRadiansPerSecond;
        public double accelerationRadiansPerSecondSq;
        public double appliedVolts;
        public double currentAmps;
    }

    default void updateInputs(WristRollIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void runOpenLoop(double percent) {}

    default void stop() {}

    default void resetPosition(double radians) {}
}
