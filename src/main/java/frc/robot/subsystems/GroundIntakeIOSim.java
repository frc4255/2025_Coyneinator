package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

/**
 * Simple physics-lite simulation of the ground intake. Tracks the pitch joint angle and roller
 * surface speed so that the subsystem and AdvantageKit logging behave similarly to the real robot.
 */
public class GroundIntakeIOSim implements GroundIntakeIO {
    private static final double MAX_PITCH_SPEED_RAD_PER_SEC = Math.toRadians(90.0);
    private static final double MAX_ROLLER_SPEED_RPS = 80.0;

    private double pitchPositionRadians;
    private double pitchVelocityRadiansPerSecond;
    private double rollerVelocityRotationsPerSecond;
    private double pitchAppliedVolts;
    private double rollerAppliedVolts;

    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.02;
        }
        lastTimestamp = now;

        pitchVelocityRadiansPerSecond = (pitchAppliedVolts / 12.0) * MAX_PITCH_SPEED_RAD_PER_SEC;
        pitchPositionRadians += pitchVelocityRadiansPerSecond * dt;

        rollerVelocityRotationsPerSecond = (rollerAppliedVolts / 12.0) * MAX_ROLLER_SPEED_RPS;

        inputs.pitchPositionRadians = pitchPositionRadians;
        inputs.pitchVelocityRadiansPerSecond = pitchVelocityRadiansPerSecond;
        inputs.rollerVelocityRotationsPerSecond = rollerVelocityRotationsPerSecond;
        inputs.pitchAppliedVolts = pitchAppliedVolts;
        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.pitchCurrentAmps = Math.abs(pitchAppliedVolts) * 5.0 / 12.0;
        inputs.rollerCurrentAmps = Math.abs(rollerAppliedVolts) * 8.0 / 12.0;
    }

    @Override
    public void setPitchVolts(double volts) {
        pitchAppliedVolts = volts;
    }

    @Override
    public void setRollerVolts(double volts) {
        rollerAppliedVolts = volts;
    }

    @Override
    public void stop() {
        setPitchVolts(0.0);
        setRollerVolts(0.0);
    }
}
