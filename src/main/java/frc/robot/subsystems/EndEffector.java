package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    
    private TalonFX motor;

    private VoltageOut motorRequest;

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

    public void setDutyCycle(double Voltage) {
        motor.setControl(motorRequest.withOutput(Voltage));
    }

    public void stop() {
        motor.stopMotor();
    }
}
