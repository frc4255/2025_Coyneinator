package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class DifferentialWristIOReal implements DifferentialWristIO {
    private static final double MOTOR_TO_DIFF_GEAR_RATIO =
        Constants.Wrist.DIFFERENTIAL_MOTOR_TO_DIFF_GEAR_RATIO;
    private static final double PITCH_DIFF_ROTATIONS_TO_RADIANS = (2.0 * Math.PI)
        / Constants.Wrist.DIFFERENTIAL_PITCH_GEAR_RATIO;
    private static final double ROLL_DIFF_ROTATIONS_TO_RADIANS = (2.0 * Math.PI)
        / Constants.Wrist.DIFFERENTIAL_ROLL_GEAR_RATIO;
    private static final double RADIANS_TO_PITCH_DIFF_ROTATIONS = 1.0
        / PITCH_DIFF_ROTATIONS_TO_RADIANS;
    private static final double RADIANS_TO_ROLL_DIFF_ROTATIONS = 1.0
        / ROLL_DIFF_ROTATIONS_TO_RADIANS;

    private final TalonFX leftMotor = new TalonFX(Constants.Wrist.DIFFERENTIAL_LEFT_MOTOR_ID);
    private final TalonFX rightMotor = new TalonFX(Constants.Wrist.DIFFERENTIAL_RIGHT_MOTOR_ID);
    private final CANcoder leftEncoder = new CANcoder(Constants.Wrist.DIFFERENTIAL_LEFT_ENCODER_ID);
    private final CANcoder rightEncoder = new CANcoder(Constants.Wrist.DIFFERENTIAL_RIGHT_ENCODER_ID);

    private final VoltageOut leftVoltageRequest = new VoltageOut(0.0);
    private final VoltageOut rightVoltageRequest = new VoltageOut(0.0);
    private final DutyCycleOut leftDutyCycleRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut rightDutyCycleRequest = new DutyCycleOut(0.0);

    public DifferentialWristIOReal() {
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);
        leftMotor.setPosition(0.0);
        rightMotor.setPosition(0.0);
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(DifferentialWristIOInputs inputs) {
        double leftMotorRotations = leftMotor.getPosition().getValueAsDouble();
        double rightMotorRotations = rightMotor.getPosition().getValueAsDouble();
        double leftMotorVelocity = leftMotor.getVelocity().getValueAsDouble();
        double rightMotorVelocity = rightMotor.getVelocity().getValueAsDouble();

        // Convert motor sensor rotations into differential bevel rotations before decoupling.
        double leftDifferentialRotations = leftMotorRotations / MOTOR_TO_DIFF_GEAR_RATIO;
        double rightDifferentialRotations = rightMotorRotations / MOTOR_TO_DIFF_GEAR_RATIO;
        double leftDifferentialVelocity = leftMotorVelocity / MOTOR_TO_DIFF_GEAR_RATIO;
        double rightDifferentialVelocity = rightMotorVelocity / MOTOR_TO_DIFF_GEAR_RATIO;

        double pitchDifferentialRotations = 0.5
            * (leftDifferentialRotations + rightDifferentialRotations);
        double rollDifferentialRotations = 0.5
            * (leftDifferentialRotations - rightDifferentialRotations);
        double pitchDifferentialVelocity = 0.5
            * (leftDifferentialVelocity + rightDifferentialVelocity);
        double rollDifferentialVelocity = 0.5
            * (leftDifferentialVelocity - rightDifferentialVelocity);

        inputs.pitchPositionRadians = pitchDifferentialRotations * PITCH_DIFF_ROTATIONS_TO_RADIANS;
        inputs.rollPositionRadians = rollDifferentialRotations * ROLL_DIFF_ROTATIONS_TO_RADIANS;
        inputs.pitchVelocityRadiansPerSecond = pitchDifferentialVelocity
            * PITCH_DIFF_ROTATIONS_TO_RADIANS;
        inputs.rollVelocityRadiansPerSecond = rollDifferentialVelocity
            * ROLL_DIFF_ROTATIONS_TO_RADIANS;

        double leftEncoderRotations = leftEncoder.getPosition().getValueAsDouble();
        double rightEncoderRotations = rightEncoder.getPosition().getValueAsDouble();
        double leftEncoderVelocity = leftEncoder.getVelocity().getValueAsDouble();
        double rightEncoderVelocity = rightEncoder.getVelocity().getValueAsDouble();

        inputs.leftPositionRotations = leftMotorRotations;
        inputs.rightPositionRotations = rightMotorRotations;
        inputs.leftVelocityRotationsPerSecond = leftMotorVelocity;
        inputs.rightVelocityRotationsPerSecond = rightMotorVelocity;
        inputs.leftEncoderPositionRotations = leftEncoderRotations;
        inputs.rightEncoderPositionRotations = rightEncoderRotations;
        inputs.leftEncoderVelocityRotationsPerSecond = leftEncoderVelocity;
        inputs.rightEncoderVelocityRotationsPerSecond = rightEncoderVelocity;
        inputs.leftAppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightAppliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrentAmps = leftMotor.getStatorCurrent().getValueAsDouble();
        inputs.rightCurrentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setMotorVoltages(double leftVolts, double rightVolts) {
        leftMotor.setControl(leftVoltageRequest.withOutput(leftVolts));
        rightMotor.setControl(rightVoltageRequest.withOutput(rightVolts));
    }

    @Override
    public void runPitchOpenLoop(double percent) {
        leftDutyCycleRequest.Output = percent;
        rightDutyCycleRequest.Output = percent;
        leftMotor.setControl(leftDutyCycleRequest);
        rightMotor.setControl(rightDutyCycleRequest);
    }

    @Override
    public void runRollOpenLoop(double percent) {
        leftDutyCycleRequest.Output = percent;
        rightDutyCycleRequest.Output = -percent;
        leftMotor.setControl(leftDutyCycleRequest);
        rightMotor.setControl(rightDutyCycleRequest);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void resetPitchPosition(double radians) {
        double desiredPitchDifferentialRotations = radians * RADIANS_TO_PITCH_DIFF_ROTATIONS;
        double currentLeftMotorRotations = leftMotor.getPosition().getValueAsDouble();
        double currentRightMotorRotations = rightMotor.getPosition().getValueAsDouble();
        double currentLeftDifferentialRotations = currentLeftMotorRotations / MOTOR_TO_DIFF_GEAR_RATIO;
        double currentRightDifferentialRotations = currentRightMotorRotations / MOTOR_TO_DIFF_GEAR_RATIO;
        double currentRollDifferentialRotations = 0.5
            * (currentLeftDifferentialRotations - currentRightDifferentialRotations);

        double newLeftDifferentialRotations = desiredPitchDifferentialRotations
            + currentRollDifferentialRotations;
        double newRightDifferentialRotations = desiredPitchDifferentialRotations
            - currentRollDifferentialRotations;

        double newLeftMotorRotations = newLeftDifferentialRotations * MOTOR_TO_DIFF_GEAR_RATIO;
        double newRightMotorRotations = newRightDifferentialRotations * MOTOR_TO_DIFF_GEAR_RATIO;

        leftMotor.setPosition(newLeftMotorRotations);
        rightMotor.setPosition(newRightMotorRotations);
        leftEncoder.setPosition(newLeftDifferentialRotations);
        rightEncoder.setPosition(newRightDifferentialRotations);
    }

    @Override
    public void resetRollPosition(double radians) {
        double desiredRollDifferentialRotations = radians * RADIANS_TO_ROLL_DIFF_ROTATIONS;
        double currentLeftMotorRotations = leftMotor.getPosition().getValueAsDouble();
        double currentRightMotorRotations = rightMotor.getPosition().getValueAsDouble();
        double currentLeftDifferentialRotations = currentLeftMotorRotations / MOTOR_TO_DIFF_GEAR_RATIO;
        double currentRightDifferentialRotations = currentRightMotorRotations / MOTOR_TO_DIFF_GEAR_RATIO;
        double currentPitchDifferentialRotations = 0.5
            * (currentLeftDifferentialRotations + currentRightDifferentialRotations);

        double newLeftDifferentialRotations = currentPitchDifferentialRotations
            + desiredRollDifferentialRotations;
        double newRightDifferentialRotations = currentPitchDifferentialRotations
            - desiredRollDifferentialRotations;

        double newLeftMotorRotations = newLeftDifferentialRotations * MOTOR_TO_DIFF_GEAR_RATIO;
        double newRightMotorRotations = newRightDifferentialRotations * MOTOR_TO_DIFF_GEAR_RATIO;

        leftMotor.setPosition(newLeftMotorRotations);
        rightMotor.setPosition(newRightMotorRotations);
        leftEncoder.setPosition(newLeftDifferentialRotations);
        rightEncoder.setPosition(newRightDifferentialRotations);
    }
}
