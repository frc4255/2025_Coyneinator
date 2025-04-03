package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
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


public class Pivot extends SubsystemBase {
    
    private TalonFX m_leftMotor;
    private TalonFX m_rightMotor;

    private VoltageOut m_leftMotorRequest = new VoltageOut(0.0);
    private VoltageOut m_Motor1Request = new VoltageOut(0.0);

    private ArmFeedforward elevatorPivotFeedforward;

    private ProfiledPIDController m_PIDController;

    private boolean isHomed = false;
    private boolean needsHoming = false;

    private boolean isPosePossible = true;

    private boolean isStowed = false;

    private DataLog log;




    public Pivot() {
        m_PIDController = new ProfiledPIDController(
            10, 
            0, 
            0, 
            new TrapezoidProfile.Constraints(
                2.5, //TODO tune this
                7 // TODO tune this
            )
        );

        m_leftMotor = new TalonFX(1);
        m_rightMotor = new TalonFX(0);
        m_leftMotor.setNeutralMode(NeutralModeValue.Coast);
        m_rightMotor.setNeutralMode(NeutralModeValue.Coast);
        m_rightMotor.setInverted(true);
        m_leftMotor.setControl(new Follower(m_rightMotor.getDeviceID(), true));
        elevatorPivotFeedforward = new ArmFeedforward(0,0.18,4.7,0);

        m_rightMotor.setPosition(0);
        setGoal(0);

        m_PIDController.setTolerance(0.25);

    }

    public void setVoltageForClimb() {
        m_rightMotor.setVoltage(-9);
    }

    public double velocityOfMotors() {
        return m_rightMotor.getVelocity().getValueAsDouble();
    }


    public void isStowed(boolean currentState) {
        isStowed = currentState;
    }

    public void setAutoHome(boolean request) {
        isHomed = request;
    }

    public void autoHome() {
        m_leftMotor.set(-0.2);
        m_rightMotor.set(-0.2);

        System.err.println("Pivot Motor Current: " + getMotorCurrent());
        System.err.println("Pivot LEft motor Velocty: " + m_leftMotor.getVelocity().getValueAsDouble());
        System.err.println("Pivot right Motor Velocty: " + m_rightMotor.getVelocity().getValueAsDouble());

        Logger.recordOutput("Pivot Motor Current", getMotorCurrent());
        Logger.recordOutput("Pivot LEft motor Velocty ", m_leftMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Pivot right Motor Velocty", m_rightMotor.getVelocity().getValueAsDouble());


        if (m_rightMotor.getVelocity().getValueAsDouble() <= 0.05) {//TODO this is def wrong.
            m_leftMotor.setPosition(0);   
            m_rightMotor.setPosition(0);

            m_leftMotor.stopMotor();
            m_rightMotor.stopMotor();

            setAutoHome(true);
            System.err.println("Pivot Homed");
        }
    }

    public double getMotorCurrent() {
        return m_rightMotor.getStatorCurrent().getValueAsDouble();
    }

    protected double getMeasurement() {
        return getPivotPosition();
    }

    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    
        double finalOut = output + elevatorPivotFeedforward.calculate(setpoint.position, setpoint.velocity);

        SmartDashboard.putNumber("setpoint velocity", setpoint.velocity);
        SmartDashboard.putNumber("Setpoint position", setpoint.position);

        Logger.recordOutput("setpoint velocity", setpoint.velocity);
        Logger.recordOutput("Setpoint position", setpoint.position);
        m_rightMotor.setControl(m_Motor1Request.withOutput(finalOut));

    }

    /* 
    public double getArmPosition() {
        return ((m_leftMotor.getPosition().getValueAsDouble()) / 161.290322581)* (2 * Math.PI); //TODO get GEAR RATIO PLEASE
    } */

    public double getPivotPosition() {
        return (m_rightMotor.getPosition().getValueAsDouble()/252)*2*Math.PI;
    }

    public void setPivotAsHomed() {
        m_leftMotor.setPosition(0.0);
        m_rightMotor.setPosition(0.0);
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

    public boolean atGoal() {
        boolean x = (Math.abs(m_PIDController.getPositionError()) < 0.05) && (m_PIDController.getSetpoint().position == m_PIDController.getGoal().position);
        System.out.println(x+"Pivot");
        return x;
    }

    public void stopMotors() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    @Override
    public void periodic() {
        super.periodic();

        double currentPosition = getPivotPosition(); 
        double pidOutput = m_PIDController.calculate(currentPosition); 

        useOutput(pidOutput, m_PIDController.getSetpoint());

        SmartDashboard.putNumber("PivotPosition", getPivotPosition());
        SmartDashboard.putNumber("Pivot velocity", ((m_rightMotor.getVelocity().getValueAsDouble())*2*Math.PI)/252);
        SmartDashboard.putNumber("Pivot Acceleration", ((m_rightMotor.getAcceleration().getValueAsDouble()/252)*2*Math.PI));
        SmartDashboard.putNumber("Pivot Motors Applied Voltage", m_rightMotor.getMotorVoltage().getValueAsDouble());
        
        Logger.recordOutput("PivotPosition", getPivotPosition());
        Logger.recordOutput("Pivot velocity", ((m_rightMotor.getVelocity().getValueAsDouble())*2*Math.PI)/252);
        Logger.recordOutput("Pivot Acceleration", ((m_rightMotor.getAcceleration().getValueAsDouble()/252)*2*Math.PI));
        Logger.recordOutput("Pivot Motors Applied Voltage", m_rightMotor.getMotorVoltage().getValueAsDouble());
        
        if (getPivotPosition() > Constants.Elevator.PivotMaxLimit ||
            getPivotPosition() < Constants.Elevator.PivotMinLimit) {
                isPosePossible = false;
            } else {
                isPosePossible = true;
            }

    }
}
