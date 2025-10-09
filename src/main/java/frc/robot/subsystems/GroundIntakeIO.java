package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/**
 * Abstraction for the ground intake hardware. Currently provides just enough surface area to
 * satisfy the higher level subsystems while the mechanism is under development.
 */
public interface GroundIntakeIO {
    @AutoLog
    public static class GroundIntakeIOInputs {
        public double pitchPositionRadians;
        public double pitchVelocityRadiansPerSecond;
        public double rollerVelocityRotationsPerSecond;
        public double pitchAppliedVolts;
        public double rollerAppliedVolts;
        public double pitchCurrentAmps;
        public double rollerCurrentAmps;
    }

    default void updateInputs(GroundIntakeIOInputs inputs) {}

    default void setPitchVolts(double volts) {}

    default void setRollerVolts(double volts) {}

    default void stop() {}

    default void resetPosition(double pos) {}
}
