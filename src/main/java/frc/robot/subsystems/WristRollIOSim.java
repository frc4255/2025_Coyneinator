package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class WristRollIOSim implements WristRollIO {
    private static final double MAX_VELOCITY = 8.0; // rad/s
    private static final double MAX_ACCELERATION = 45.0; // rad/s^2
    private static final double DAMPING_COEFFICIENT = 3.0;
    private static final double POSITION_LIMIT = Math.PI;

    private double positionRadians = 0.0;
    private double velocityRadiansPerSecond = 0.0;
    private double appliedVolts = 0.0;
    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(WristRollIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.02;
        }
        lastTimestamp = now;

        double acceleration = (appliedVolts / 12.0) * MAX_ACCELERATION
            - velocityRadiansPerSecond * DAMPING_COEFFICIENT;

        velocityRadiansPerSecond += acceleration * dt;
        velocityRadiansPerSecond = MathUtil.clamp(
            velocityRadiansPerSecond,
            -MAX_VELOCITY,
            MAX_VELOCITY
        );

        positionRadians += velocityRadiansPerSecond * dt;
        positionRadians = MathUtil.clamp(positionRadians, -POSITION_LIMIT, POSITION_LIMIT);

        inputs.positionRadians = positionRadians;
        inputs.velocityRadiansPerSecond = velocityRadiansPerSecond;
        inputs.accelerationRadiansPerSecondSq = acceleration;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(appliedVolts) / 12.0 * 25.0;
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
    public void resetPosition(double radians) {
        positionRadians = MathUtil.clamp(radians, -POSITION_LIMIT, POSITION_LIMIT);
        velocityRadiansPerSecond = 0.0;
    }
}
