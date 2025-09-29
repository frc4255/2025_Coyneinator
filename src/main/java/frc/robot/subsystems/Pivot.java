package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Pivot extends SubsystemBase {

    private static final double HOMING_VELOCITY_THRESHOLD_RAD_PER_SEC = 0.05;
    private static final double HOMING_VOLTAGE = -0.2 * 12.0;
    private static final double CLIMB_VOLTAGE = -9.0;

    private final PivotIO io;
    private final PivotIO.PivotIOInputs inputs = new PivotIO.PivotIOInputs();

    private final ArmFeedforward elevatorPivotFeedforward;

    private final ProfiledPIDController m_PIDController;

    private boolean isHomed = false;
    private boolean needsHoming = false;
    private boolean isPosePossible = true;
    private boolean isStowed = false;

    public Pivot(PivotIO io) {
        this.io = io;
        m_PIDController = new ProfiledPIDController(
            Constants.Elevator.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                4,
                5
            )
        );
        m_PIDController.setTolerance(0.25);

        elevatorPivotFeedforward = new ArmFeedforward(
            Constants.Elevator.Pivot.kS,
            Constants.Elevator.Pivot.kG,
            Constants.Elevator.Pivot.kV,
            Constants.Elevator.Pivot.kA
        );
    }

    public void setVoltageForClimb() {
        io.setVoltage(CLIMB_VOLTAGE);
    }

    public double velocityOfMotors() {
        return inputs.velocityRadsPerSec;
    }

    public void isStowed(boolean currentState) {
        isStowed = currentState;
    }

    public boolean isStowed() {
        return isStowed;
    }

    public void setAutoHome(boolean request) {
        isHomed = request;
        needsHoming = !request;
    }

    public void autoHome() {
        io.updateInputs(inputs);
        io.setVoltage(HOMING_VOLTAGE);

        Logger.recordOutput("Pivot/HomingCurrent", getMotorCurrent());
        Logger.recordOutput("Pivot/HomingVelocity", inputs.velocityRadsPerSec);

        if (Math.abs(inputs.velocityRadsPerSec) <= HOMING_VELOCITY_THRESHOLD_RAD_PER_SEC) {
            io.resetPosition(0.0);
            io.stop();
            setAutoHome(true);
        }
    }

    public double getMotorCurrent() {
        return inputs.statorCurrentAmps;
    }

    public double getPivotPosition() {
        return inputs.positionRads;
    }

    public void setPivotAsHomed() {
        io.resetPosition(0.0);
        isHomed = true;
        needsHoming = false;
    }

    public boolean needsHoming() {
        return needsHoming;
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

    public boolean atGoal() {
        boolean atGoal = Math.abs(m_PIDController.getPositionError()) < 0.05
            && m_PIDController.getSetpoint().position == m_PIDController.getGoal().position;
        Logger.recordOutput("Pivot/AtGoal", atGoal);
        return atGoal;
    }

    public void stopMotors() {
        io.stop();
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);

        double currentPosition = getPivotPosition();
        double pidOutput = m_PIDController.calculate(currentPosition);

        double finalOut = pidOutput + elevatorPivotFeedforward.calculate(
            m_PIDController.getSetpoint().position,
            m_PIDController.getSetpoint().velocity
        );

        io.setVoltage(finalOut);

        SmartDashboard.putNumber("PivotPosition", inputs.positionRads);
        SmartDashboard.putNumber("Pivot velocity", inputs.velocityRadsPerSec);
        SmartDashboard.putNumber("Pivot Acceleration", inputs.accelerationRadsPerSecSq);
        SmartDashboard.putNumber("Pivot Motors Applied Voltage", inputs.appliedVolts);

        Logger.recordOutput("Pivot/Position", inputs.positionRads);
        Logger.recordOutput("Pivot/Velocity", inputs.velocityRadsPerSec);
        Logger.recordOutput("Pivot/Acceleration", inputs.accelerationRadsPerSecSq);
        Logger.recordOutput("Pivot/AppliedVolts", inputs.appliedVolts);
        Logger.recordOutput("Pivot/StatorCurrent", inputs.statorCurrentAmps);

        if (inputs.positionRads > Constants.Elevator.PivotMaxLimit ||
            inputs.positionRads < Constants.Elevator.PivotMinLimit) {
            isPosePossible = false;
        } else {
            isPosePossible = true;
        }
    }
}
