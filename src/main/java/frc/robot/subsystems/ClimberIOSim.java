package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

public class ClimberIOSim implements ClimberIO {
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(appliedVolts) / 12.0 * 35.0;
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
    }
}
