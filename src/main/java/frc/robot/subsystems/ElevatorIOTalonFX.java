package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

/**
 * Real-hardware implementation of {@link ElevatorIO} backed by TalonFX motor
 * controllers.
 */
public class ElevatorIOTalonFX implements ElevatorIO {
    private static final double METERS_PER_MOTOR_ROTATION = 0.0375;

    private final TalonFX leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
    private final TalonFX rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);

    private final VoltageOut rightMotorRequest = new VoltageOut(0.0);
    private final Follower leftFollowerRequest;

    public ElevatorIOTalonFX() {
        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        rightMotor.setInverted(true);
        leftFollowerRequest = new Follower(rightMotor.getDeviceID(), true);
        leftMotor.setControl(leftFollowerRequest);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = rightMotor.getPosition().getValueAsDouble() * METERS_PER_MOTOR_ROTATION;
        inputs.velocityMetersPerSecond = rightMotor.getVelocity().getValueAsDouble() * METERS_PER_MOTOR_ROTATION;
        inputs.accelerationMetersPerSecondSq = rightMotor.getAcceleration().getValueAsDouble() * METERS_PER_MOTOR_ROTATION;
        inputs.appliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        rightMotor.setControl(rightMotorRequest.withOutput(volts));
        leftMotor.setControl(leftFollowerRequest);
    }

    @Override
    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    @Override
    public void resetPosition(double positionMeters) {
        double motorRotations = positionMeters / METERS_PER_MOTOR_ROTATION;
        rightMotor.setPosition(motorRotations);
        leftMotor.setPosition(motorRotations);
    }
}
