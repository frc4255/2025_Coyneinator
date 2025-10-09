package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.PivotIOInputsAutoLogged;

public class Pivot extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private final ArmFeedforward feedforward;
    private final ProfiledPIDController controller;

    private boolean isHomed = false;
    private boolean isPosePossible = true;
    private boolean isStowed = false;
    private double goalRadians = 0.0;

    public Pivot(PivotIO io) {
        this.io = io;

        controller = new ProfiledPIDController(
            10,
            0,
            0,
            new TrapezoidProfile.Constraints(2.5, 7)
        );

        feedforward = new ArmFeedforward(0, 0.18, 4.7, 0);

        controller.setTolerance(0.25);
        setGoal(0.0);
    }

    public void setVoltageForClimb() {
        io.setVoltage(-9.0);
    }

    public double velocityOfMotors() {
        return inputs.velocityRadiansPerSecond;
    }

    public void isStowed(boolean currentState) {
        isStowed = currentState;
    }

    public void setAutoHome(boolean request) {
        isHomed = request;
    }

    public void autoHome() {
        io.runOpenLoop(-0.2);

        Logger.recordOutput("Pivot/HomingVelocity", inputs.velocityRadiansPerSecond);
        Logger.recordOutput("Pivot/HomingCurrent", inputs.currentAmps);

        if (Math.abs(inputs.velocityRadiansPerSecond) <= 0.05) {
            io.resetPosition(0.0);
            io.stop();
            setAutoHome(true);
        }
    }

    public double getMotorCurrent() {
        return inputs.currentAmps;
    }

    public double getPivotPosition() {
        return inputs.positionRadians;
    }

    public void setPivotAsHomed() {
        io.resetPosition(0.0);
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setGoal(double pos) {
        goalRadians = pos;
        controller.setGoal(pos);
    }

    public boolean isPivotPosePossible() {
        return isPosePossible;
    }

    public boolean atGoal() {
        return Math.abs(controller.getPositionError()) < 0.05
            && controller.getSetpoint().position == controller.getGoal().position;
    }

    public void stopMotors() {
        io.stop();
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double finalOut = output + feedforward.calculate(setpoint.position, setpoint.velocity);
        io.setVoltage(finalOut);
        Logger.recordOutput("Pivot/AppliedVoltage", finalOut);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        double currentPosition = getPivotPosition();
        double pidOutput = controller.calculate(currentPosition);
        useOutput(pidOutput, controller.getSetpoint());

        Logger.recordOutput("Pivot/Position", currentPosition);
        Logger.recordOutput("Pivot/GoalPosition", goalRadians);
        Logger.recordOutput("Pivot/SetpointPosition", controller.getSetpoint().position);
        Logger.recordOutput("Pivot/Velocity", inputs.velocityRadiansPerSecond);
        Logger.recordOutput("Pivot/Acceleration", inputs.accelerationRadiansPerSecondSq);

        if (currentPosition > Constants.Elevator.PivotMaxLimit
            || currentPosition < Constants.Elevator.PivotMinLimit) {
            isPosePossible = false;
        } else {
            isPosePossible = true;
        }
    }
}
