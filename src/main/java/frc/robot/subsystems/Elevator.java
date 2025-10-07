package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PivotingElevatorFeedforward;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorIOInputsAutoLogged;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final PivotingElevatorFeedforward feedforward;
    private final ProfiledPIDController controller;

    private boolean isHomed = false;
    private boolean active = false;

    private final DoubleSupplier pivotAngleSupplier;

    public Elevator(ElevatorIO io, DoubleSupplier pivotAngleSupplier) {
        this.io = io;
        this.pivotAngleSupplier = pivotAngleSupplier;

        feedforward = new PivotingElevatorFeedforward(0.35, 3.5, 0.01);
        controller = new ProfiledPIDController(
            12,
            0,
            0,
            new TrapezoidProfile.Constraints(
                2,
                2
            )
        );

        controller.setTolerance(0.3);
        setGoal(0.0);
    }

    public void setActive() {
        active = true;
    }

    public void setAutoHome(boolean request) {
        isHomed = request;
    }

    public void autoHome() {
        io.runOpenLoop(-0.3);

        Logger.recordOutput("Elevator/HomingVelocity", inputs.velocityMetersPerSecond);
        Logger.recordOutput("Elevator/HomingCurrent", inputs.currentAmps);

        if (Math.abs(inputs.velocityMetersPerSecond) <= 0.05) {
            io.resetPosition(0.0);
            io.stop();
            setAutoHome(true);
        }
    }

    public boolean isHomed() {
        return isHomed;
    }

    public double getMotorCurrent() {
        return inputs.currentAmps;
    }

    public double getElevatorPosition() {
        return inputs.positionMeters;
    }

    public void setElevatorAsHomed() {
        io.resetPosition(0.0);
    }

    public void setGoal(double pos) {
        controller.setGoal(pos);
    }

    public boolean atGoal() {
        return Math.abs(controller.getPositionError()) < 0.1
            && controller.getSetpoint().position == controller.getGoal().position;
    }

    public void stopMotors() {
        io.stop();
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double totalOutput =
            output
                + feedforward.calculate(
                    setpoint.velocity,
                    setpoint.position,
                    pivotAngleSupplier.getAsDouble()
                );
        io.setVoltage(totalOutput);
        Logger.recordOutput("Elevator/AppliedVoltage", totalOutput);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (active) {
            double measurement = getElevatorPosition();
            double pidOutput = controller.calculate(measurement);
            useOutput(pidOutput, controller.getSetpoint());
        }

        Logger.recordOutput("Elevator/GoalPosition", controller.getGoal().position);
        Logger.recordOutput("Elevator/PositionMeters", getElevatorPosition());
        Logger.recordOutput("Elevator/VelocityMetersPerSecond", inputs.velocityMetersPerSecond);
        Logger.recordOutput("Elevator/AccelerationMetersPerSecondSq", inputs.accelerationMetersPerSecondSq);
    }
}
