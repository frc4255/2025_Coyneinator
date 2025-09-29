package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class EndEffectorIOReal implements EndEffectorIO {
    private final TalonFX motor = new TalonFX(Constants.Wrist.END_EFFECTOR_MOTOR_ID);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public EndEffectorIOReal() {
        motor.getConfigurator().apply(
            new TalonFXConfiguration().OpenLoopRamps.withVoltageOpenLoopRampPeriod(0.25)
        );
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
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
