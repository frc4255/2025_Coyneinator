package frc.robot.subsystems;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

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
            1, 
            0, 
            0,
            new TrapezoidProfile.Constraints(
                4,
                4
            )
        );
        
        feedforward = new ArmFeedforward(
            0,
            0,
            0
        );

        pitchClosedLoopEnabled = false;
    }

    public void setPitchVolts(double volts) {
        io.setPitchVolts(volts);
        pitchClosedLoopEnabled = false;
        Logger.recordOutput("GroundIntake/PitchCommandVolts", volts);
    }

    public void setRollerVolts(double volts) {
        io.setRollerVolts(volts);
        Logger.recordOutput("GroundIntake/RollerCommandVolts", volts);
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
        io.setPitchVolts(applied);
        Logger.recordOutput("GroundIntake/PIDVolts", pidVolts);
        Logger.recordOutput("GroundIntake/FeedforwardVolts", ffVolts);
        Logger.recordOutput("GroundIntake/PitchControllerOutputVolts", applied);
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
