package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private double lastCommandVolts = 0.0;

    public Climber() {
        motor = new TalonFX(Constants.Climber.MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Climber.OPEN_LOOP_RAMP_SECONDS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Climber.CURRENT_LIMIT_AMPS;
        motor.getConfigurator().apply(config);
    }

    public void setVoltage(double volts) {
        lastCommandVolts = volts;
        motor.setControl(voltageRequest.withOutput(volts));
    }

    public double getMotorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void stop() {
        lastCommandVolts = 0.0;
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/CommandVolts", lastCommandVolts);
        Logger.recordOutput("Climber/StatorCurrent", getMotorCurrent());
        Logger.recordOutput(
            "Climber/AppliedVolts",
            motor.getMotorVoltage().getValueAsDouble()
        );
    }
}
