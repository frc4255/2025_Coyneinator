package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO {
    private static final double ROTATIONS_TO_METERS = 0.0375;

    private final TalonFX leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID, "rio");
    private final TalonFX rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID, "rio");

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    public ElevatorIOReal() {
        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        rightMotor.setInverted(true);
        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));

        rightMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = rightMotor.getPosition().getValueAsDouble() * ROTATIONS_TO_METERS;
        inputs.velocityMetersPerSecond = rightMotor.getVelocity().getValueAsDouble() * ROTATIONS_TO_METERS;
        inputs.accelerationMetersPerSecondSq = rightMotor.getAcceleration().getValueAsDouble() * ROTATIONS_TO_METERS;
        inputs.appliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        rightMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void runOpenLoop(double percent) {
        dutyCycleRequest.Output = percent;
        rightMotor.setControl(dutyCycleRequest);
    }

    @Override
    public void stop() {
        rightMotor.stopMotor();
    }

    @Override
    public void resetPosition(double positionMeters) {
        double rotations = positionMeters / ROTATIONS_TO_METERS;
        leftMotor.setPosition(rotations);
        rightMotor.setPosition(rotations);
    }
}
