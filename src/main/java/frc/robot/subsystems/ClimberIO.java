package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double appliedVolts;
        public double currentAmps;
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void stop() {}
}
