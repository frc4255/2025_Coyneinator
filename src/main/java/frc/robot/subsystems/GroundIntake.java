package frc.robot.subsystems;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple placeholder ground intake subsystem so the rest of the codebase can compile while the
 * mechanism implementation is in flux. All methods currently forward directly to the provided IO
 * layer.
 */
public class GroundIntake extends SubsystemBase {
    private final GroundIntakeIO io;
    private final GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();

    public GroundIntake(GroundIntakeIO io) {
        this.io = Objects.requireNonNull(io);
    }

    public void setPitchVolts(double volts) {
        io.setPitchVolts(volts);
    }

    public void setRollerVolts(double volts) {
        io.setRollerVolts(volts);
    }

    public void stop() {
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("GroundIntake", inputs);
    }
}
