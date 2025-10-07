package frc.robot.subsystems;

public class DifferentialWristIOSim implements DifferentialWristIO {
    private double pitchRadians;
    private double rollRadians;

    @Override
    public void updateInputs(DifferentialWristIOInputs inputs) {
        inputs.pitchPositionRadians = pitchRadians;
        inputs.pitchVelocityRadiansPerSecond = 0.0;
        inputs.rollPositionRadians = rollRadians;
        inputs.rollVelocityRadiansPerSecond = 0.0;
        inputs.leftEncoderPositionRotations = 0.0;
        inputs.rightEncoderPositionRotations = 0.0;
        inputs.leftEncoderVelocityRotationsPerSecond = 0.0;
        inputs.rightEncoderVelocityRotationsPerSecond = 0.0;
        inputs.leftAppliedVolts = 0.0;
        inputs.rightAppliedVolts = 0.0;
        inputs.leftCurrentAmps = 0.0;
        inputs.rightCurrentAmps = 0.0;
    }

    @Override
    public void setMotorVoltages(double leftVolts, double rightVolts) {
        // Simple approximation: treat left/right symmetry and update pose directly.
        double pitchDelta = (leftVolts + rightVolts) * 0.0005;
        double rollDelta = (leftVolts - rightVolts) * 0.0005;
        pitchRadians += pitchDelta;
        rollRadians += rollDelta;
    }

    @Override
    public void runPitchOpenLoop(double percent) {
        double pitchDelta = percent * 0.01;
        pitchRadians += pitchDelta;
    }

    @Override
    public void runRollOpenLoop(double percent) {
        double rollDelta = percent * 0.01;
        rollRadians += rollDelta;
    }

    @Override
    public void stop() {
        // Nothing to do for the simple sim implementation.
    }

    @Override
    public void resetPitchPosition(double radians) {
        pitchRadians = radians;
    }

    @Override
    public void resetRollPosition(double radians) {
        rollRadians = radians;
    }
}
