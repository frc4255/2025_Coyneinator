package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotStateMachine;

public class Elevator extends ProfiledPIDSubsystem {
    
    private TalonFX m_LeftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
    private TalonFX m_RightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);

    private VoltageOut m_LeftMotorRequest = new VoltageOut(0.0);
    private VoltageOut m_rightMotorRequest = new VoltageOut(0.0);

    private StateManager s_RobotState = new StateManager();

    private boolean isHomed = false;

    public Elevator() {
        super(new ProfiledPIDController(
            Constants.Elevator.kP, 
            0,
            0,
            new TrapezoidProfile.Constraints(15, 23))
        );

        m_LeftMotor.setNeutralMode(NeutralModeValue.Brake);
        m_RightMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override
    protected double getMeasurement() {
        return getElevatorPosition();
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_LeftMotor.setControl(m_LeftMotorRequest.withOutput(output));
        m_RightMotor.setControl(m_rightMotorRequest.withOutput(output));
    }


    public double getElevatorPosition() {
        return ((m_LeftMotor.getPosition().getValueAsDouble()) / 32767.99976); //normalize values to 0-1
    }

    public void setIntakeAsHomed() {
        m_LeftMotor.setPosition(0.0);
        m_RightMotor.setPosition(0.0);
        isHomed = true;
    }

    public void setPos(double pos) {
       setGoal(pos);
    }

}
