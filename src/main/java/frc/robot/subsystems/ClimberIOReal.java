package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {
    private final TalonFX motor = new TalonFX(Constants.Climber.MOTOR_ID);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
