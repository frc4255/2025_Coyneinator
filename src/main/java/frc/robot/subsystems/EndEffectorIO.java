package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorIOInputs {
        public double appliedVolts;
        public double currentAmps;
    }

    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void stop() {}
}
