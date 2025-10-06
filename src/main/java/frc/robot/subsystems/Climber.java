package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
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

public class Climber extends SubsystemBase {
    
    private TalonFX motor;

    private VoltageOut motorRequest;

    public Climber() {
        motor = new TalonFX(Constants.Climber.MOTOR_ID);
        motorRequest = new VoltageOut(0);
    }

    public double getMotorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void setVoltage(double Voltage) {
        motor.setControl(motorRequest.withOutput(Voltage));
    }

    public void stop() {
        motor.stopMotor();
    }
}
