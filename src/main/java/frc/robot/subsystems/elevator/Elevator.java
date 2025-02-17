package frc.robot.subsystems.elevator;

import java.util.HashMap;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotStateMachine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends SubsystemBase {
    
    private TalonFX m_LeftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
    private TalonFX m_RightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);

    private VoltageOut m_LeftMotorRequest = new VoltageOut(0.0);
    private VoltageOut m_rightMotorRequest = new VoltageOut(0.0);

    private StateManager s_RobotState = new StateManager();

    private ProfiledPIDController m_PIDController;

    private boolean isHomed = false;

    private boolean isPosePossible = true;

    public Elevator() {
        m_PIDController = new ProfiledPIDController(
            Constants.Elevator.kP, 
            0, 
            0, 
            new TrapezoidProfile.Constraints(
                4, //TODO tune this
                5 // TODO tune this
            )
        );

        m_LeftMotor.setNeutralMode(NeutralModeValue.Brake);
        m_RightMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    protected double getMeasurement() {
        return getElevatorPosition();
    }

    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_LeftMotor.setControl(m_LeftMotorRequest.withOutput(output));
        m_RightMotor.setControl(m_rightMotorRequest.withOutput(output));
    }


    public double getElevatorPosition() {
        return ((m_LeftMotor.getPosition().getValueAsDouble())); //TODO FIND ELEVATOR MAX HEIGHT!!!
    }

    public void setElevatorAsHomed() {
        m_LeftMotor.setPosition(0.0);
        m_RightMotor.setPosition(0.0);
        isHomed = true;
    }

    public void setPos(double pos) {
       m_PIDController.setGoal(pos);
    }

    public boolean isElevatorPosePossible() {
        return isPosePossible;
    }

    @Override
    public void periodic() {
        super.periodic();

        double currentPosition = getMeasurement(); 
        double pidOutput = m_PIDController.calculate(currentPosition); 

        useOutput(pidOutput, m_PIDController.getSetpoint());

        SmartDashboard.putNumber("ElevatorPosition", getElevatorPosition());

        if(getElevatorPosition() > Constants.Elevator.ElevatorMaxExtensionLimit ||
                getElevatorPosition() < Constants.Elevator.ElevatorMinExtensionLimit) {
            isPosePossible = false;
        } else {
            isPosePossible = true;
        }
    }
}