package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PivotingElevatorFeedforward;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ElevatorFeedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends SubsystemBase {
    
    private TalonFX m_LeftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID, "rio");
    private TalonFX m_RightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID, "rio");

    private VoltageOut m_LeftMotorRequest = new VoltageOut(0.0);
    private VoltageOut m_rightMotorRequest = new VoltageOut(0.0);

    private PivotingElevatorFeedforward m_Feedforward;
    private ProfiledPIDController m_PIDController;

    private boolean isHomed = false;

    private boolean isPosePossible = true;

    private boolean active = false;

    private DoubleSupplier pivotAngleSupplier;

    public Elevator(DoubleSupplier pivotAngleSupplier) {

        m_Feedforward = new PivotingElevatorFeedforward(
            1,
            3.1,
            0.01
        );

        m_PIDController = new ProfiledPIDController(
            1, 
            0, 
            0, 
            new TrapezoidProfile.Constraints(
                3, //TODO tune this
                2 // TODO tune this
            )
        );

        this.pivotAngleSupplier = pivotAngleSupplier;

        
        m_LeftMotor.setNeutralMode(NeutralModeValue.Brake);
        m_RightMotor.setNeutralMode(NeutralModeValue.Brake);

        m_RightMotor.setInverted(true);
        m_LeftMotor.setControl(new Follower(m_RightMotor.getDeviceID(), true));

        m_RightMotor.setPosition(0);
        setGoal(0);

        m_PIDController.setTolerance(0.3);
    }

    protected double getMeasurement() {
        return getElevatorPosition();
    }

    protected void useOutput(double output, TrapezoidProfile.State setpoint) {

        SmartDashboard.putNumber("elevator setpoint velocity", setpoint.velocity);
        SmartDashboard.putNumber("elevator Setpoint position", setpoint.position);

        m_RightMotor.setControl(m_rightMotorRequest.withOutput(output));
    }

    public void setActive() {
        active = true;
    }


    //Returns in meters
    // Math is pitch diameter (32T HTD 5mm = 2.005 smthn in, divided by 1000, all over 4 (gear reduction))
    public double getElevatorPosition() {
        return (m_RightMotor.getPosition().getValueAsDouble()*0.04); 
    }

    public void setElevatorAsHomed() {
        m_LeftMotor.setPosition(0.0);
        m_RightMotor.setPosition(0.0);
        isHomed = true;
    }

    public void setGoal(double pos) {
       m_PIDController.setGoal(pos);
    }

    public boolean atGoal() {
        boolean x = (Math.abs(m_PIDController.getPositionError()) < 0.05) && (m_PIDController.getSetpoint().position == m_PIDController.getGoal().position);
        System.out.println(x+"Elevator");
        return x;
    }

    public void stopMotors() {
        m_RightMotor.stopMotor();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (active) {
            double totalOutput = m_PIDController.calculate(getElevatorPosition()) +
                m_Feedforward.calculate(
                    m_PIDController.getSetpoint().velocity,
                    m_PIDController.getSetpoint().position,
                    pivotAngleSupplier.getAsDouble()
                ); 

            useOutput(totalOutput, m_PIDController.getSetpoint());
        }
        SmartDashboard.putNumber("Elevator Goal", m_PIDController.getVelocityError());
        SmartDashboard.putNumber("ElevatorPosition", getElevatorPosition());
        SmartDashboard.putNumber("Elevator velocity", (m_RightMotor.getVelocity().getValueAsDouble()*0.04));
        SmartDashboard.putNumber("Elevator Acceleration", (m_RightMotor.getAcceleration().getValueAsDouble()*0.04));
        SmartDashboard.putNumber("Elevator Motors Applied Voltage", m_RightMotor.getMotorVoltage().getValueAsDouble());
    }
}