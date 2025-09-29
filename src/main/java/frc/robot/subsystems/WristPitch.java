package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristPitch extends SubsystemBase {

    private static final double HOMING_VOLTAGE = 0.04 * 12.0;
    private static final double HOMING_VELOCITY_THRESHOLD_RAD_PER_SEC = 0.05;

    private final WristPitchIO io;
    private final WristPitchIO.WristPitchIOInputs inputs = new WristPitchIO.WristPitchIOInputs();

    private final ProfiledPIDController m_PIDController;
    private final ArmFeedforward feedforward;

    private boolean isHomed = false;
    private boolean isPosePossible = true;
    private boolean active = false;

    public WristPitch(WristPitchIO io) {
        this.io = io;
        m_PIDController = new ProfiledPIDController(
            Constants.Wrist.Pitch_kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                8,
                8
            )
        );
        m_PIDController.setTolerance(0.3);

        feedforward = new ArmFeedforward(
            Constants.Wrist.kS,
            Constants.Wrist.kG,
            Constants.Wrist.kV
        );
    }

    public void setActive() {
        active = true;
    }

    public void setActive(boolean request) {
        active = request;
    }

    public void setAutoHome(boolean request) {
        isHomed = request;
    }

    public void autoHome() {
        io.updateInputs(inputs);
        io.setVoltage(HOMING_VOLTAGE);

        Logger.recordOutput("WristPitch/HomingVelocity", inputs.velocityRadsPerSec);
        Logger.recordOutput("WristPitch/HomingCurrent", inputs.statorCurrentAmps);

        if (Math.abs(inputs.velocityRadsPerSec) <= HOMING_VELOCITY_THRESHOLD_RAD_PER_SEC) {
            io.resetPosition(0.0);
            io.stop();
            setAutoHome(true);
        }
    }

    public double getCurrentPos() {
        return inputs.positionRads;
    }

    public void setHomed() {
        io.resetPosition(0.0);
        isHomed = true;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public double getMotorCurrent() {
        return inputs.statorCurrentAmps;
    }

    public void setGoal(double pos) {
       m_PIDController.setGoal(pos);
       setActive(true);
    }

    public boolean isPivotPosePossible() {
        return isPosePossible;
    }

    public void stopMotors() {
        io.stop();
    }

    public boolean atGoal() {
        boolean atGoal = Math.abs(m_PIDController.getPositionError()) < 0.05
            && m_PIDController.getSetpoint().position == m_PIDController.getGoal().position;
        Logger.recordOutput("WristPitch/AtGoal", atGoal);
        return atGoal;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);

        if (active) {
            double currentPosition = getCurrentPos();
            double pidOutput = m_PIDController.calculate(currentPosition);

            double finalOut = pidOutput + feedforward.calculate(
                m_PIDController.getSetpoint().position,
                m_PIDController.getSetpoint().velocity
            );

            io.setVoltage(finalOut);
        }

        if (inputs.positionRads > Constants.Wrist.PitchMaxLimit ||
            inputs.positionRads < Constants.Wrist.PitchMinLimit) {
            isPosePossible = false;
        } else {
            isPosePossible = true;
        }

        SmartDashboard.putNumber("Wrist Error", m_PIDController.getVelocityError());
        SmartDashboard.putNumber("Wrist Pitch", inputs.positionRads);
        SmartDashboard.putNumber("Wrist velocity", inputs.velocityRadsPerSec);
        SmartDashboard.putNumber("Wrist Acceleration", inputs.accelerationRadsPerSecSq);
        SmartDashboard.putNumber("Wrist Motors Applied Voltage", inputs.appliedVolts);

        Logger.recordOutput("WristPitch/Error", m_PIDController.getVelocityError());
        Logger.recordOutput("WristPitch/Position", inputs.positionRads);
        Logger.recordOutput("WristPitch/Velocity", inputs.velocityRadsPerSec);
        Logger.recordOutput("WristPitch/Acceleration", inputs.accelerationRadsPerSecSq);
        Logger.recordOutput("WristPitch/AppliedVolts", inputs.appliedVolts);
        Logger.recordOutput("WristPitch/StatorCurrent", inputs.statorCurrentAmps);
    }
}
