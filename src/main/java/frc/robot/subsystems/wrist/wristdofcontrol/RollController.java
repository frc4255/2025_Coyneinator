package frc.robot.subsystems.wrist.wristdofcontrol;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollController extends SubsystemBase {
    private final TalonFX m_Motor0;

    private final PIDController rollPitchPID = new PIDController(Constants.Wrist.Roll_kP, 0, 0); //TODO tune this

    public RollController(int RollMotorID) {
        m_Motor0 = new TalonFX(Constants.Wrist.ROLL_MOTOR_ID);

        m_Motor0.setNeutralMode(NeutralModeValue.Brake);

    }

    public void setRollAngleRAD(double targetAngle) {
        double output = rollPitchPID.calculate(getCurrentAngle(), targetAngle);
        m_Motor0.set(output);
    }

    public double getCurrentAngle() {
        return m_Motor0.getPosition().getValueAsDouble() * 2 * Math.PI; //TODO get gear ration 0.0

    }

    public void Stop() {
        m_Motor0.stopMotor();
    }

    public void SetHomeRollWristPitchMotor() {
        m_Motor0.setPosition(0);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber("Wrist Roll Angle in Radians", getCurrentAngle());
    }
}
