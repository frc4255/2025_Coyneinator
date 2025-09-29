package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PivotingElevatorFeedforward;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private static final double HOMING_VOLTAGE = -0.3 * 12.0;
    private static final double HOMING_VELOCITY_THRESHOLD_METERS_PER_SEC = 0.05;

    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

    private final PivotingElevatorFeedforward feedforward;
    private final ProfiledPIDController controller;

    private boolean isHomed = false;
    private boolean needsHoming = false;
    private boolean isPosePossible = true;
    private boolean active = false;

    private final DoubleSupplier pivotAngleSupplier;

    public Elevator(ElevatorIO io, DoubleSupplier pivotAngleSupplier) {
        this.io = io;
        this.pivotAngleSupplier = pivotAngleSupplier;

        feedforward = new PivotingElevatorFeedforward(
            Constants.Elevator.kG,
            Constants.Elevator.kV,
            Constants.Elevator.kA
        );

        controller = new ProfiledPIDController(
            Constants.Elevator.kP,
            Constants.Elevator.kI,
            Constants.Elevator.kD,
            new TrapezoidProfile.Constraints(
                Constants.Elevator.MAX_VEL,
                Constants.Elevator.MAX_ACC
            )
        );
        controller.setTolerance(0.3);
    }

    public void setActive() {
        active = true;
    }

    public void setActive(boolean request) {
        active = request;
    }

    public void setInactive() {
        active = false;
        io.stop();
    }

    public void setAutoHome(boolean request) {
        isHomed = request;
        needsHoming = !request;
    }

    public void autoHome() {
        io.updateInputs(inputs);
        io.setVoltage(HOMING_VOLTAGE);

        Logger.recordOutput("Elevator/HomingVelocity", inputs.velocityMetersPerSecond);
        Logger.recordOutput("Elevator/HomingCurrent", inputs.statorCurrentAmps);

        if (Math.abs(inputs.velocityMetersPerSecond) <= HOMING_VELOCITY_THRESHOLD_METERS_PER_SEC) {
            io.resetPosition(0.0);
            io.stop();
            setAutoHome(true);
        }
    }

    public boolean isHomed() {
        return isHomed;
    }

    public boolean needsHoming() {
        return needsHoming;
    }

    public double getMotorCurrent() {
        return inputs.statorCurrentAmps;
    }

    //Returns in meters
    // Math is pitch diameter (48T HTD 5mm = 70 smthn mm, divided by 1000, all over 4 (gear reduction))
    public double getElevatorPosition() {
        return inputs.positionMeters;
    }

    public void setElevatorAsHomed() {
        io.resetPosition(0.0);
        isHomed = true;
        needsHoming = false;
    }

    public void setGoal(double pos) {
       controller.setGoal(pos);
       setActive(true);
    }

    public boolean atGoal() {
        boolean atGoal = Math.abs(controller.getPositionError()) < 0.1
            && controller.getSetpoint().position == controller.getGoal().position;
        Logger.recordOutput("Elevator/AtGoal", atGoal);
        return atGoal;
    }

    public boolean isPosePossible() {
        return isPosePossible;
    }

    public void stopMotors() {
        io.stop();
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);

        if (active) {
            double totalOutput = controller.calculate(getElevatorPosition()) +
                feedforward.calculate(
                    controller.getSetpoint().velocity,
                    controller.getSetpoint().position,
                    pivotAngleSupplier.getAsDouble()
                );

            io.setVoltage(totalOutput);
        }

        if (inputs.positionMeters > Constants.Elevator.ElevatorMaxExtensionLimit ||
            inputs.positionMeters < Constants.Elevator.ElevatorMinExtensionLimit) {
            isPosePossible = false;
        } else {
            isPosePossible = true;
        }

        SmartDashboard.putNumber("Elevator Goal", controller.getVelocityError());
        SmartDashboard.putNumber("ElevatorPosition", inputs.positionMeters);
        SmartDashboard.putNumber("Elevator velocity", inputs.velocityMetersPerSecond);
        SmartDashboard.putNumber("Elevator Acceleration", inputs.accelerationMetersPerSecondSq);
        SmartDashboard.putNumber("Elevator Motors Applied Voltage", inputs.appliedVolts);

        Logger.recordOutput("Elevator/GoalVelocityError", controller.getVelocityError());
        Logger.recordOutput("Elevator/Position", inputs.positionMeters);
        Logger.recordOutput("Elevator/Velocity", inputs.velocityMetersPerSecond);
        Logger.recordOutput("Elevator/Acceleration", inputs.accelerationMetersPerSecondSq);
        Logger.recordOutput("Elevator/AppliedVolts", inputs.appliedVolts);
        Logger.recordOutput("Elevator/StatorCurrent", inputs.statorCurrentAmps);
    }
}