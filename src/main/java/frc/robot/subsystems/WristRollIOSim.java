package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/** Simulation implementation of {@link WristRollIO}. */
public class WristRollIOSim implements WristRollIO {
    private static final double MAX_ACCEL_UNITS_PER_SECOND_SQ = 10.0;

    private double positionRads;
    private double velocityRadsPerSec;
    private double appliedVoltage;
    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(WristRollIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        double acceleration = (appliedVoltage / 12.0) * MAX_ACCEL_UNITS_PER_SECOND_SQ;
        velocityRadsPerSec += acceleration * dt;
        positionRads += velocityRadsPerSec * dt;

        inputs.positionRads = positionRads;
        inputs.velocityRadsPerSec = velocityRadsPerSec;
        inputs.accelerationRadsPerSecSq = acceleration;
        inputs.appliedVolts = appliedVoltage;
        inputs.statorCurrentAmps = Math.abs(appliedVoltage) / 12.0 * 20.0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void stop() {
        appliedVoltage = 0.0;
        velocityRadsPerSec = 0.0;
    }

    @Override
    public void resetPosition(double positionRads) {
        this.positionRads = positionRads;
        this.velocityRadsPerSec = 0.0;
    }
}
