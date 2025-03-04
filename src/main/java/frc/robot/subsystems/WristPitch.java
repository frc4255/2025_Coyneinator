package frc.robot.subsystems;

import java.util.HashMap;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotStateMachine;

public class WristPitch extends SubsystemBase {
    
    private TalonFX m_Motor0 = new TalonFX(Constants.Elevator.PIVOT_LEFT_MOTOR_ID);
    private TalonFX m_Motor1 = new TalonFX(Constants.Elevator.PIVOT_RIGHT_MOTOR_ID);

    private VoltageOut m_Motor0Request = new VoltageOut(0.0);
    private VoltageOut m_Motor1Request = new VoltageOut(0.0);

    //TODO: Custom Feedforward Controller

    private ProfiledPIDController m_PIDController;

    private boolean isHomed = false;

    private boolean isPosePossible = true;

    private DataLog log;

    public WristPitch() {
        m_PIDController = new ProfiledPIDController(
            Constants.Elevator.kP, 
            0, 
            0, 
            new TrapezoidProfile.Constraints(
                4, //TODO tune this
                5 // TODO tune this
            )
        );

        m_Motor0.setNeutralMode(NeutralModeValue.Brake);
        m_Motor1.setNeutralMode(NeutralModeValue.Brake);
    }

    protected double getMeasurement() {
        return getCurrentPos();
    }

    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    
        double finalOut = output;
        
        m_Motor0.setControl(m_Motor0Request.withOutput(finalOut));
        m_Motor1.setControl(m_Motor1Request.withOutput(finalOut));

    }

    public double getCurrentPos() {
        return m_Motor0.getPosition().getValueAsDouble(); //TODO: Gear Ratio
    }

    public void setHomed() {
        m_Motor0.setPosition(0.0);
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
        m_Motor0.stopMotor();
        m_Motor1.stopMotor();
    }

    public boolean atGoal() {
        return atGoal();
    }

    @Override
    public void periodic() {
        super.periodic();

        double currentPosition = getMeasurement(); 
        double pidOutput = m_PIDController.calculate(currentPosition); 

        useOutput(pidOutput, m_PIDController.getSetpoint());

        SmartDashboard.putNumber("WristPitch", getCurrentPos());
    }
}
