package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface DifferentialWristIO {
    @AutoLog
    public static class DifferentialWristIOInputs {
        public double pitchPositionRadians;
        public double pitchVelocityRadiansPerSecond;
        public double rollPositionRadians;
        public double rollVelocityRadiansPerSecond;

        public double leftPositionRotations;
        public double rightPositionRotations;
        public double leftVelocityRotationsPerSecond;
        public double rightVelocityRotationsPerSecond;

        public double leftEncoderPositionRotations;
        public double rightEncoderPositionRotations;
        public double leftEncoderVelocityRotationsPerSecond;
        public double rightEncoderVelocityRotationsPerSecond;

        public double leftAppliedVolts;
        public double rightAppliedVolts;
        public double leftCurrentAmps;
        public double rightCurrentAmps;
    }

    default void updateInputs(DifferentialWristIOInputs inputs) {}

    default void setMotorVoltages(double leftVolts, double rightVolts) {}

    default void runPitchOpenLoop(double percent) {}

    default void runRollOpenLoop(double percent) {}

    default void stop() {}

    default void resetPitchPosition(double radians) {}

    default void resetRollPosition(double radians) {}
}
