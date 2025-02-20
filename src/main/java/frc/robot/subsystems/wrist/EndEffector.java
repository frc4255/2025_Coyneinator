package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase{
    private TalonFX m_Motor0 = new TalonFX(Constants.Wrist.END_EFFECTOR_MOTOR_ID);

    public EndEffector() {
        m_Motor0.setNeutralMode(NeutralModeValue.Brake);
    }

    public void runEndEffector() {
        m_Motor0.set(0.625); //TODO tune this
    }

    public void idleEndEffector() {
        m_Motor0.set(0.01); //TODO tune this
    }

    public void inverseIdleEndEffector() {
        m_Motor0.set(-0.01); //TODO tune this
    }

    public void inverseRunEndEffector() {
        m_Motor0.set(-0.625); //TODO tune this
    }

    public void stop() {
        m_Motor0.stopMotor();
    }
}
