package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristRoll extends SubsystemBase {

    private final WristRollIO io;
    private final WristRollIO.WristRollIOInputs inputs = new WristRollIO.WristRollIOInputs();

    private final ProfiledPIDController m_PIDController;

    private boolean isHomed = false;
    private boolean isPosePossible = true;
    private boolean active = false;

    public WristRoll(WristRollIO io) {
        this.io = io;
        m_PIDController = new ProfiledPIDController(
            Constants.Wrist.Roll_kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                10,
                12
            )
        );

        m_PIDController.setTolerance(0.2);
    }

    public void controlManually(double request) {
        io.setVoltage(MathUtil.clamp(request, -1.0, 1.0) * 12.0);
    }

    public void setActive() {
        active = true;
    }

    public void setActive(boolean request) {
        active = request;
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

    public boolean atGoal() {
        boolean atGoal = Math.abs(m_PIDController.getPositionError()) < 0.02
            && m_PIDController.getSetpoint().position == m_PIDController.getGoal().position;
        Logger.recordOutput("WristRoll/AtGoal", atGoal);
        return atGoal;
    }

    public boolean isPivotPosePossible() {
        return isPosePossible;
    }

    public void stopMotor() {
        io.stop();
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);

        if (active) {
            double currentPosition = getCurrentPos();
            double pidOutput = m_PIDController.calculate(currentPosition);

            io.setVoltage(pidOutput);
        }

        if (inputs.positionRads > Constants.Wrist.RollMaxLimit ||
            inputs.positionRads < Constants.Wrist.RollMinLimit) {
            isPosePossible = false;
        } else {
            isPosePossible = true;
        }

        SmartDashboard.putNumber("WristRoll", inputs.positionRads);
        SmartDashboard.putNumber("WristRollVelocity", inputs.velocityRadsPerSec);
        SmartDashboard.putNumber("WristRollAcceleration", inputs.accelerationRadsPerSecSq);
        SmartDashboard.putNumber("WristRollAppliedVoltage", inputs.appliedVolts);

        Logger.recordOutput("WristRoll/Position", inputs.positionRads);
        Logger.recordOutput("WristRoll/Velocity", inputs.velocityRadsPerSec);
        Logger.recordOutput("WristRoll/Acceleration", inputs.accelerationRadsPerSecSq);
        Logger.recordOutput("WristRoll/AppliedVolts", inputs.appliedVolts);
        Logger.recordOutput("WristRoll/StatorCurrent", inputs.statorCurrentAmps);
    }
}
