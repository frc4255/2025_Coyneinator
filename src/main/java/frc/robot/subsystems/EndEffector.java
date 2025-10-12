package frc.robot.subsystems;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndEffectorIOInputsAutoLogged;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public EndEffector(EndEffectorIO io) {
        this.io = Objects.requireNonNull(io, "end effector IO cannot be null");
    }

    public double getMotorCurrent() {
        return inputs.currentAmps;
    }

    public void setDutyCycle(double volts) {
        double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        io.setVoltage(appliedVolts);
        Logger.recordOutput("EndEffector/CommandedVolts", appliedVolts);
    }

    public void stop() {
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);
    }
}
