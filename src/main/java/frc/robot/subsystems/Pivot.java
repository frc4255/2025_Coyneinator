package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.PivotIOInputsAutoLogged;

public class Pivot extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private final ArmFeedforward feedforward;
    private final ProfiledPIDController controller;

    private boolean isHomed = false;
    private boolean isPosePossible = true;
    private boolean isStowed = false;

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
        double position = inputs.positionRadians;
        if (Robot.isReal()) {
            position -= Constants.Elevator.Pivot.ZERO_OFFSET_RADIANS;
        }
        return position;
    }

    public double getSetpointPosition() {
        return controller.getSetpoint().position;
    }

    public double getGoalPosition() {
        return controller.getGoal().position;
    }

    public void setPivotAsHomed() {
        io.resetPosition(0.0);
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setGoal(double pos) {
        controller.setGoal(pos);
        Logger.recordOutput("Pivot/GoalPosition", pos);
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
        double ffVolts = feedforward.calculate(setpoint.position, setpoint.velocity);
        double finalOut = output + ffVolts;
        io.setVoltage(finalOut);
        Logger.recordOutput("Pivot/PIDOutputVolts", output);
        Logger.recordOutput("Pivot/FeedforwardVolts", ffVolts);
        Logger.recordOutput("Pivot/AppliedVoltage", finalOut);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        double currentPosition = getPivotPosition();
        Logger.recordOutput("Pivot/MeasurementRadians", currentPosition);
        double pidOutput = controller.calculate(currentPosition);
        TrapezoidProfile.State setpoint = controller.getSetpoint();
        Logger.recordOutput("Pivot/PositionError", controller.getPositionError());
        Logger.recordOutput("Pivot/SetpointPosition", setpoint.position);
        Logger.recordOutput("Pivot/SetpointVelocity", setpoint.velocity);
        useOutput(pidOutput, setpoint);

        Logger.recordOutput("Pivot/Position", currentPosition);
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
