package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class WristPitchIOReal implements WristPitchIO {
    private static final double ROTATIONS_TO_RADIANS = (2.0 * Math.PI) / 51.0;

    private final TalonFX motor = new TalonFX(Constants.Wrist.PITCH_MOTOR_ID);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    public WristPitchIOReal() {
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setPosition(0.0);
    }

    @Override
    public void updateInputs(WristPitchIOInputs inputs) {
        inputs.positionRadians = motor.getPosition().getValueAsDouble() * ROTATIONS_TO_RADIANS;
        inputs.velocityRadiansPerSecond = motor.getVelocity().getValueAsDouble() * ROTATIONS_TO_RADIANS;
        inputs.accelerationRadiansPerSecondSq = motor.getAcceleration().getValueAsDouble() * ROTATIONS_TO_RADIANS;
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void runOpenLoop(double percent) {
        dutyCycleRequest.Output = percent;
        motor.setControl(dutyCycleRequest);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void resetPosition(double radians) {
        double rotations = radians / ROTATIONS_TO_RADIANS;
        motor.setPosition(rotations);
    }
}
