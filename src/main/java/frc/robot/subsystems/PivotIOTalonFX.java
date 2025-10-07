package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

/** Real-hardware implementation of {@link PivotIO}. */
public class PivotIOTalonFX implements PivotIO {
    private static final double ROTATIONS_TO_RADIANS = 2.0 * Math.PI / 252.0;

    private final TalonFX leftMotor = new TalonFX(Constants.Elevator.PIVOT_LEFT_MOTOR_ID);
    private final TalonFX rightMotor = new TalonFX(Constants.Elevator.PIVOT_RIGHT_MOTOR_ID);

    private final VoltageOut leaderRequest = new VoltageOut(0.0);
    private final Follower followerRequest;

    public PivotIOTalonFX() {
        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        rightMotor.setInverted(true);
        followerRequest = new Follower(rightMotor.getDeviceID(), true);
        leftMotor.setControl(followerRequest);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        double rotations = rightMotor.getPosition().getValueAsDouble();
        double velocityRotPerSec = rightMotor.getVelocity().getValueAsDouble();
        double accelRotPerSecSq = rightMotor.getAcceleration().getValueAsDouble();

        inputs.positionRads = rotations * ROTATIONS_TO_RADIANS;
        inputs.velocityRadsPerSec = velocityRotPerSec * ROTATIONS_TO_RADIANS;
        inputs.accelerationRadsPerSecSq = accelRotPerSecSq * ROTATIONS_TO_RADIANS;
        inputs.appliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        rightMotor.setControl(leaderRequest.withOutput(volts));
        leftMotor.setControl(followerRequest);
    }

    @Override
    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    @Override
    public void resetPosition(double positionRads) {
        double motorRotations = positionRads / ROTATIONS_TO_RADIANS;
        leftMotor.setPosition(motorRotations);
        rightMotor.setPosition(motorRotations);
    }
}
