package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simple physics-lite simulation for the elevator that satisfies the
 * {@link ElevatorIO} contract.
 */
public class ElevatorIOSim implements ElevatorIO {
    private static final double MAX_ACCEL_METERS_PER_SECOND_SQ = 3.0;

    private double positionMeters;
    private double velocityMetersPerSecond;
    private double appliedVoltage;
    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        double acceleration = (appliedVoltage / 12.0) * MAX_ACCEL_METERS_PER_SECOND_SQ;
        velocityMetersPerSecond += acceleration * dt;
        positionMeters += velocityMetersPerSecond * dt;

        inputs.positionMeters = positionMeters;
        inputs.velocityMetersPerSecond = velocityMetersPerSecond;
        inputs.accelerationMetersPerSecondSq = acceleration;
        inputs.appliedVolts = appliedVoltage;
        inputs.statorCurrentAmps = Math.abs(appliedVoltage) / 12.0 * 40.0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void stop() {
        appliedVoltage = 0.0;
        velocityMetersPerSecond = 0.0;
    }

    @Override
    public void resetPosition(double positionMeters) {
        this.positionMeters = positionMeters;
        this.velocityMetersPerSecond = 0.0;
    }
}
