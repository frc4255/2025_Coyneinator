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

public class WristRoll extends SubsystemBase {
    
    private TalonFX m_Motor0 = new TalonFX(Constants.Wrist.ROLL_MOTOR_ID);

    private VoltageOut m_Motor0Request = new VoltageOut(0.0);

    //TODO: Custom Feedforward Controller

    private ProfiledPIDController m_PIDController;

    private boolean isHomed = false;

    private boolean isPosePossible = true;

    private boolean active = false;
    
    private DataLog log;

    public WristRoll() {
        m_PIDController = new ProfiledPIDController(
            15, 
            0, 
            0, 
            new TrapezoidProfile.Constraints(
                10, //TODO tune this
                12 // TODO tune this
            )
        );

        m_Motor0.setNeutralMode(NeutralModeValue.Coast);

        m_Motor0.setPosition(0);
    }

    protected double getMeasurement() {
        return getCurrentPos();
    }

    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    
        double finalOut = output;
        
        m_Motor0.setControl(m_Motor0Request.withOutput(finalOut));

    }

    public double getCurrentPos() {
        return (m_Motor0.getPosition().getValueAsDouble()/60)*2*Math.PI; //TODO: Gear Ratio
    }

    public void setHomed() {
        m_Motor0.setPosition(0.0);
        isHomed = true;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setGoal(double pos) {
       m_PIDController.setGoal(pos);
    }

    public boolean atGoal() {
        return atGoal();
    }

    public boolean isPivotPosePossible() {
        return isPosePossible;
    }

    public void stopMotors() {
        m_Motor0.stopMotor();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (active) {
            double currentPosition = getMeasurement(); 
            double pidOutput = m_PIDController.calculate(currentPosition); 

            useOutput(pidOutput, m_PIDController.getSetpoint());
        }
        SmartDashboard.putNumber("WristRoll", getCurrentPos());
    }
}
