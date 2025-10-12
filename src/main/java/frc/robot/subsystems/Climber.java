package frc.robot.subsystems;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClimberIOInputsAutoLogged;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = Objects.requireNonNull(io, "climber IO cannot be null");
    }

    public double getMotorCurrent() {
        return inputs.currentAmps;
    }

    public void setVoltage(double voltage) {
        double clampedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        io.setVoltage(clampedVoltage);
        Logger.recordOutput("Climber/CommandedVolts", clampedVoltage);
    }

    public void stop() {
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}
