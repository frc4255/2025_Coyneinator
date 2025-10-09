package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

/**
 * Real hardware binding for the ground intake. Maps the TalonFXs that control the pitch joints and
 * intake roller into the {@link GroundIntakeIO} abstraction used by the subsystem.
 */
public class GroundIntakeIOReal implements GroundIntakeIO {
    private static final double ROTATIONS_TO_RADIANS =
            2.0 * Math.PI / Math.max(Constants.GroundIntake.PITCH_GEAR_RATIO, 1e-6);

    private final TalonFX pitchLeader =
            new TalonFX(Constants.GroundIntake.PITCH_LEADER_MOTOR_ID);
    private final TalonFX pitchFollower =
            new TalonFX(Constants.GroundIntake.PITCH_FOLLOWER_MOTOR_ID);
    private final TalonFX rollerMotor =
            new TalonFX(Constants.GroundIntake.ROLLER_MOTOR_ID);

    private final VoltageOut pitchVoltageRequest = new VoltageOut(0.0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0);

    public GroundIntakeIOReal() {
        pitchLeader.setNeutralMode(NeutralModeValue.Brake);
        pitchLeader.setPosition(0.0);

        pitchFollower.setNeutralMode(NeutralModeValue.Brake);
        pitchFollower.setControl(new Follower(pitchLeader.getDeviceID(), false));
        pitchFollower.setPosition(0.0);

        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(GroundIntakeIOInputs inputs) {
        inputs.pitchPositionRadians =
                pitchLeader.getPosition().getValueAsDouble() * ROTATIONS_TO_RADIANS;
        inputs.pitchVelocityRadiansPerSecond =
                pitchLeader.getVelocity().getValueAsDouble() * ROTATIONS_TO_RADIANS;
        inputs.pitchAppliedVolts = pitchLeader.getMotorVoltage().getValueAsDouble();
        inputs.pitchCurrentAmps = pitchLeader.getStatorCurrent().getValueAsDouble();

        inputs.rollerVelocityRotationsPerSecond = rollerMotor.getVelocity().getValueAsDouble();
        inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollerCurrentAmps = rollerMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setPitchVolts(double volts) {
        pitchLeader.setControl(pitchVoltageRequest.withOutput(volts));
    }

    @Override
    public void setRollerVolts(double volts) {
        rollerMotor.setControl(rollerVoltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        pitchLeader.stopMotor();
        pitchFollower.stopMotor();
        rollerMotor.stopMotor();
    }
}
