package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

/** Real-hardware implementation of {@link WristRollIO}. */
public class WristRollIOTalonFX implements WristRollIO {
    private static final double ROTATIONS_TO_RADIANS = 2.0 * Math.PI / 60.0;

    private final TalonFX motor = new TalonFX(Constants.Wrist.ROLL_MOTOR_ID);
    private final VoltageOut request = new VoltageOut(0.0);

    public WristRollIOTalonFX() {
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void updateInputs(WristRollIOInputs inputs) {
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
