package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Grabber extends SubsystemBase{
    
    private TalonFX m_motor0 = new TalonFX(Constants.Grabber.MOTOR_ID_0);

    public Grabber() {
        m_motor0.setNeutralMode(NeutralModeValue.Brake);
    }

    public void RunGrabber() {
        m_motor0.set(0.5); //TODO Tune this
    }
}
