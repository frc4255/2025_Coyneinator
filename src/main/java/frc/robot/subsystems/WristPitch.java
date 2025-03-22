package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristPitch extends SubsystemBase {
    
    private TalonFX m_Motor1 = new TalonFX(Constants.Wrist.PITCH_MOTOR_ID);

    private VoltageOut m_Motor0Request = new VoltageOut(0.0);
    private VoltageOut m_Motor1Request = new VoltageOut(0.0);

    //TODO: Custom Feedforward Controller

    private ProfiledPIDController m_PIDController;
    private ArmFeedforward m_Feedforward;
    private boolean isHomed = false;

    private boolean isPosePossible = true;

    private boolean active = false;
    private DataLog log;

    public WristPitch() {
        m_PIDController = new ProfiledPIDController(
            20,
            0, 
            0, 
            new TrapezoidProfile.Constraints(
                8
                , //TODO tune this
                8 // TODO tune this
            )
        );

        m_Feedforward = new ArmFeedforward(0.05, 0.0, 0.8);

        m_Motor1.setNeutralMode(NeutralModeValue.Brake);

        m_Motor1.setPosition(0);
        setGoal(0);

        m_PIDController.setTolerance(0.3);
    }

    protected double getMeasurement() {
        return getCurrentPos();
    }

    public void setActive() {
        active = true;
    }

    public void autoHome() {
        m_Motor1.set(-0.04);

        if (getMotorCurrent() >= 40 && m_Motor1.getVelocity().getValueAsDouble() <= 0.05) {//TODO this is def wrong.
            m_Motor1.setPosition(0);

            m_Motor1.stopMotor();
            }
    }

    public double getMotorCurrent() {
        return m_Motor1.getStatorCurrent().getValueAsDouble();
    }

    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    
        double finalOut = output + m_Feedforward.calculate(setpoint.position, setpoint.velocity);
        
        SmartDashboard.putNumber("Wrist P setpoint velocity", setpoint.velocity);
        SmartDashboard.putNumber("Wrist P Setpoint position", setpoint.position);

        m_Motor1.setControl(m_Motor1Request.withOutput(finalOut));

    }

    public double getCurrentPos() {
        return (m_Motor1.getPosition().getValueAsDouble()/51)*2*Math.PI; //TODO: Gear Ratio
    }

    public void setHomed() {
        m_Motor1.setPosition(0.0);
        isHomed = true;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setGoal(double pos) {
       m_PIDController.setGoal(pos);
    }

    public boolean isPivotPosePossible() {
        return isPosePossible;
    }

    public void stopMotors() {
        m_Motor1.stopMotor();
    }

    public boolean atGoal() {
        boolean x = (Math.abs(m_PIDController.getPositionError()) < 0.02) && (m_PIDController.getSetpoint().position == m_PIDController.getGoal().position);
        System.out.println(x+"Pitch");
        return x;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (active) {
            double currentPosition = getMeasurement(); 
            double pidOutput = m_PIDController.calculate(currentPosition); 

            useOutput(pidOutput, m_PIDController.getSetpoint());
        } 
        SmartDashboard.putNumber("Wrist Error", m_PIDController.getVelocityError());

        SmartDashboard.putNumber("Wrist Pitch", getCurrentPos());
        SmartDashboard.putNumber("Wrist velocity", ((m_Motor1.getVelocity().getValueAsDouble())*2*Math.PI)/51);
        SmartDashboard.putNumber("Wrist Acceleration", ((m_Motor1.getAcceleration().getValueAsDouble()/51)*2*Math.PI));
        SmartDashboard.putNumber("Wrist Motors Applied Voltage", m_Motor1.getMotorVoltage().getValueAsDouble());

    }
}
