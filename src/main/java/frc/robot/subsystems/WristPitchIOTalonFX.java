package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

/** Real-hardware implementation of {@link WristPitchIO}. */
public class WristPitchIOTalonFX implements WristPitchIO {
    private static final double ROTATIONS_TO_RADIANS = 2.0 * Math.PI / 51.0;

    private final TalonFX motor = new TalonFX(Constants.Wrist.PITCH_MOTOR_ID);
    private final VoltageOut request = new VoltageOut(0.0);

    public WristPitchIOTalonFX() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(WristPitchIOInputs inputs) {
        double rotations = motor.getPosition().getValueAsDouble();
        double velocityRotPerSec = motor.getVelocity().getValueAsDouble();
        double accelRotPerSecSq = motor.getAcceleration().getValueAsDouble();

        inputs.positionRads = rotations * ROTATIONS_TO_RADIANS;
        inputs.velocityRadsPerSec = velocityRotPerSec * ROTATIONS_TO_RADIANS;
        inputs.accelerationRadsPerSecSq = accelRotPerSecSq * ROTATIONS_TO_RADIANS;
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(request.withOutput(volts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void resetPosition(double positionRads) {
        motor.setPosition(positionRads / ROTATIONS_TO_RADIANS);
    }
}
