package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotStateMachine;

public class Arm extends ProfiledPIDSubsystem {
    
    private TalonFX m_Motor0 = new TalonFX(Constants.Arm.MOTOR_ID_0);
    private TalonFX m_Motor1 = new TalonFX(Constants.Arm.MOTOR_ID_1);

    private VoltageOut m_Motor0Request = new VoltageOut(0.0);
    private VoltageOut m_Motor1Request = new VoltageOut(0.0);

    private ArmFeedforward armFeedforward;

    private StateManager s_RobotState = new StateManager();

    private boolean isHomed = false;

    public Arm() {
        super(new ProfiledPIDController(
            Constants.Elevator.kP, 
            0,
            0,
            new TrapezoidProfile.Constraints(15, 23))
        );

        m_Motor0.setNeutralMode(NeutralModeValue.Brake);
        m_Motor1.setNeutralMode(NeutralModeValue.Brake);

        armFeedforward = new ArmFeedforward(Constants.Arm.kS, Constants.Arm.kG, 
                                            Constants.Arm.kV, Constants.Arm.kA);

    }

    @Override
    protected double getMeasurement() {
        return getArmPosition();
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    
        double finalOut = output + armFeedforward.calculate(setpoint.position, setpoint.velocity);

        m_Motor0.setControl(m_Motor0Request.withOutput(output));
        m_Motor1.setControl(m_Motor1Request.withOutput(output));

    }


    public double getArmPosition() {
        return ((m_Motor0.getPosition().getValueAsDouble()) / 161.290322581)* (2 * Math.PI); //TODO get GEAR RATIO PLEASE
    }

    public void setIntakeAsHomed() {
        m_Motor0.setPosition(0.0);
        m_Motor1.setPosition(0.0);
        isHomed = true;
    }

    public void setPos(double pos) {
       setGoal(pos);
    }

}
