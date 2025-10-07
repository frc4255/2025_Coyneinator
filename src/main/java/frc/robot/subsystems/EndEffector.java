package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

    private final TalonFX motor;
    private final VoltageOut motorRequest;
    private double lastCommandVolts = 0.0;

    public EndEffector() {
        motor = new TalonFX(6);
        motorRequest = new VoltageOut(0);

        motor.getConfigurator().apply(
            new TalonFXConfiguration().OpenLoopRamps.withVoltageOpenLoopRampPeriod(0.25)
        );
    }

    public double getMotorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void setDutyCycle(double voltage) {
        lastCommandVolts = voltage;
        motor.setControl(motorRequest.withOutput(voltage));
    }

    public void stop() {
        lastCommandVolts = 0.0;
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("EndEffector/CommandVolts", lastCommandVolts);
        Logger.recordOutput("EndEffector/StatorCurrent", getMotorCurrent());
        Logger.recordOutput(
            "EndEffector/AppliedVolts",
            motor.getMotorVoltage().getValueAsDouble()
        );
    }
}
