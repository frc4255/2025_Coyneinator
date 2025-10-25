package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class PivotIOReal implements PivotIO {
    private static final double ROTATIONS_TO_RADIANS = (2.0 * Math.PI) / 252.0;

    private final TalonFX leftMotor = new TalonFX(Constants.Elevator.PIVOT_LEFT_MOTOR_ID);
    private final TalonFX rightMotor = new TalonFX(Constants.Elevator.PIVOT_RIGHT_MOTOR_ID);

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    public PivotIOReal() {
        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setInverted(true);
        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), true));
        rightMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRadians = rightMotor.getPosition().getValueAsDouble() * ROTATIONS_TO_RADIANS;
        inputs.velocityRadiansPerSecond = rightMotor.getVelocity().getValueAsDouble() * ROTATIONS_TO_RADIANS;
        inputs.accelerationRadiansPerSecondSq = rightMotor.getAcceleration().getValueAsDouble() * ROTATIONS_TO_RADIANS;
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
        leftMotor.stopMotor();
    }

    @Override
    public void resetPosition(double radians) {
        double rotations = radians / ROTATIONS_TO_RADIANS;
        leftMotor.setPosition(rotations);
        rightMotor.setPosition(rotations);
    }
}
