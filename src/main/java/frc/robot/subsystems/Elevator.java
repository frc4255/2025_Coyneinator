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
    
    private TalonFX m_LeftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
    private TalonFX m_RightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);

    private VoltageOut m_LeftMotorRequest = new VoltageOut(0.0);
    private VoltageOut m_rightMotorRequest = new VoltageOut(0.0);

    private PivotingElevatorFeedforward m_Feedforward;
    private ProfiledPIDController m_PIDController;

    private boolean isHomed = false;

    private boolean isPosePossible = true;

    private DoubleSupplier pivotAngleSupplier;

    public Elevator(DoubleSupplier pivotAngleSupplier) {

        m_Feedforward = new PivotingElevatorFeedforward(
            Constants.Elevator.kG,
            Constants.Elevator.kV,
            Constants.Elevator.kA
        );

        m_PIDController = new ProfiledPIDController(
            Constants.Elevator.kP, 
            Constants.Elevator.kI, 
            Constants.Elevator.kD, 
            new TrapezoidProfile.Constraints(
                Constants.Elevator.MAX_VEL, //TODO tune this
                Constants.Elevator.MAX_ACC // TODO tune this
            )
        );

        this.pivotAngleSupplier = pivotAngleSupplier;

        m_LeftMotor.setNeutralMode(NeutralModeValue.Brake);
        m_RightMotor.setNeutralMode(NeutralModeValue.Brake);

        m_LeftMotor.setControl(new Follower(m_RightMotor.getDeviceID(), true));

    }

    protected double getMeasurement() {
        return getElevatorPosition();
    }

    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_RightMotor.setControl(m_rightMotorRequest.withOutput(output));
    }


    //Returns in meters
    // Math is pitch diameter (48T HTD 5mm = 70 smthn mm, divided by 1000, all over 4 (gear reduction))
    public double getElevatorPosition() {
        return ((m_RightMotor.getPosition().getValueAsDouble() * (0.0190975))); 
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
        return atGoal();
    }

    public void stopMotors() {
        m_RightMotor.stopMotor();
    }

    @Override
    public void periodic() {
        super.periodic();

        double totalOutput = m_PIDController.calculate(getElevatorPosition()) +
            m_Feedforward.calculate(
                m_PIDController.getSetpoint().velocity,
                m_PIDController.getSetpoint().position,
                pivotAngleSupplier.getAsDouble()
            ); 

        useOutput(totalOutput, m_PIDController.getSetpoint());

        SmartDashboard.putNumber("ElevatorPosition", getElevatorPosition());
    }
}