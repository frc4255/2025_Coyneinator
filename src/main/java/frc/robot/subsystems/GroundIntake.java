package frc.robot.subsystems;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Simple placeholder ground intake subsystem so the rest of the codebase can compile while the
 * mechanism implementation is in flux. All methods currently forward directly to the provided IO
 * layer.
 */
public class GroundIntake extends SubsystemBase {
    private final GroundIntakeIO io;
    private final GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();

    private final ArmFeedforward feedforward;
    private final ProfiledPIDController controller;
    private boolean pitchClosedLoopEnabled;

    public GroundIntake(GroundIntakeIO io) {
        this.io = Objects.requireNonNull(io);

        controller = new ProfiledPIDController(
            Constants.GroundIntake.PITCH_KP,
            Constants.GroundIntake.PITCH_KI,
            Constants.GroundIntake.PITCH_KD,
            new TrapezoidProfile.Constraints(
                Constants.GroundIntake.PITCH_MAX_VELOCITY_RAD_PER_SEC,
                Constants.GroundIntake.PITCH_MAX_ACCEL_RAD_PER_SEC_SQ
            )
        );
        controller.setTolerance(Constants.GroundIntake.PITCH_POSITION_TOLERANCE_RADIANS);

        feedforward = new ArmFeedforward(
            Constants.GroundIntake.PITCH_KS,
            Constants.GroundIntake.PITCH_KG,
            Constants.GroundIntake.PITCH_KV,
            Constants.GroundIntake.PITCH_KA
        );

        pitchClosedLoopEnabled = false;
    }

    public void setPitchVolts(double volts) {
        double clampedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        io.setPitchVolts(clampedVolts);
        pitchClosedLoopEnabled = false;
        Logger.recordOutput("GroundIntake/PitchCommandVolts", clampedVolts);
    }

    public void setRollerVolts(double volts) {
        double clampedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        io.setRollerVolts(clampedVolts);
        Logger.recordOutput("GroundIntake/RollerCommandVolts", clampedVolts);
    }

    public void stop() {
        io.stop();
    }

    public void setPitchGoal(double radians) {
        controller.setGoal(radians);
        Logger.recordOutput("GroundIntake/GoalPosition", radians);
        pitchClosedLoopEnabled = true;
    }

    public double getPosition() {
        double pitch = inputs.pitchPositionRadians;
        if (Robot.isReal()) {
            pitch -= Constants.GroundIntake.ZERO_OFFSET_RADIANS;
        }
        return pitch;
    }

    public double getPitchPosition() {
        return getPosition();
    }

    public double getPitchSetpointPosition() {
        return controller.getSetpoint().position;
    }

    public double getPitchGoalPosition() {
        return controller.getGoal().position;
    }

    public void setAsHomed() {
        io.resetPosition(0.0);
    }

    public boolean atGoal() {
        return Math.abs(controller.getPositionError()) < 0.05
            && controller.getSetpoint().position == controller.getGoal().position;
    }

    public double getPitchVelocity() {
        return inputs.pitchVelocityRadiansPerSecond;
    }

    private void useOutput(double pidVolts, TrapezoidProfile.State setpoint) {
        double ffVolts = feedforward.calculate(setpoint.position, setpoint.velocity);
        double applied = pidVolts + ffVolts;
        double clampedApplied = MathUtil.clamp(applied, -12.0, 12.0);
        io.setPitchVolts(clampedApplied);
        Logger.recordOutput("GroundIntake/PIDVolts", pidVolts);
        Logger.recordOutput("GroundIntake/FeedforwardVolts", ffVolts);
        Logger.recordOutput("GroundIntake/PitchControllerOutputVolts", clampedApplied);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("GroundIntake", inputs);

        double position = getPosition();
        Logger.recordOutput("GroundIntake/MeasurementPosition", position);
        Logger.recordOutput("GroundIntake/MeasurementVelocity", inputs.pitchVelocityRadiansPerSecond);
        Logger.recordOutput("GroundIntake/RollerVelocityRPS", inputs.rollerVelocityRotationsPerSecond);
        Logger.recordOutput("GroundIntake/RollerAppliedVolts", inputs.rollerAppliedVolts);
        Logger.recordOutput("GroundIntake/PitchAppliedVolts", inputs.pitchAppliedVolts);
        Logger.recordOutput("GroundIntake/PitchCurrentAmps", inputs.pitchCurrentAmps);
        Logger.recordOutput("GroundIntake/RollerCurrentAmps", inputs.rollerCurrentAmps);
        TrapezoidProfile.State setpoint = controller.getSetpoint();
        Logger.recordOutput("GroundIntake/ControllerError", controller.getPositionError());
        Logger.recordOutput("GroundIntake/SetpointPosition", setpoint.position);
        Logger.recordOutput("GroundIntake/SetpointVelocity", setpoint.velocity);

        if (pitchClosedLoopEnabled) {
            double pidOutput = controller.calculate(position);
            useOutput(pidOutput, setpoint);
        }
    }
}
