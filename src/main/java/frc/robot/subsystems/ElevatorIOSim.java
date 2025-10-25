package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorIOSim implements ElevatorIO {
    private static final double MAX_VELOCITY = 2.0; // m/s
    private static final double MAX_ACCELERATION = 6.0; // m/s^2
    private static final double DAMPING_COEFFICIENT = 1.5;

    private double positionMeters = 0.0;
    private double velocityMetersPerSecond = 0.0;
    private double appliedVolts = 0.0;
    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.02;
        }
        lastTimestamp = now;

        double acceleration = (appliedVolts / 12.0) * MAX_ACCELERATION
            - velocityMetersPerSecond * DAMPING_COEFFICIENT;

        velocityMetersPerSecond += acceleration * dt;
        velocityMetersPerSecond = MathUtil.clamp(
            velocityMetersPerSecond,
            -MAX_VELOCITY,
            MAX_VELOCITY
        );

        positionMeters += velocityMetersPerSecond * dt;

        inputs.positionMeters = positionMeters;
        inputs.velocityMetersPerSecond = velocityMetersPerSecond;
        inputs.accelerationMetersPerSecondSq = acceleration;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(appliedVolts) / 12.0 * 50.0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void runOpenLoop(double percent) {
        appliedVolts = MathUtil.clamp(percent, -1.0, 1.0) * 12.0;
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
    }

    @Override
    public void resetPosition(double positionMeters) {
        this.positionMeters = positionMeters;
        this.velocityMetersPerSecond = 0.0;
    }
}
