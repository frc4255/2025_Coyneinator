package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DifferentialWristIOInputsAutoLogged;

public class DifferentialWrist extends SubsystemBase {
    private final DifferentialWristIO io;
    private final DifferentialWristIOInputsAutoLogged inputs = new DifferentialWristIOInputsAutoLogged();

    private final ProfiledPIDController pitchController;
    private final ProfiledPIDController rollController;
    private final ArmFeedforward pitchFeedforward;

    private boolean pitchActive = false;
    private boolean rollActive = false;
    private boolean isHomed = false;
    private boolean autoHomeRequested = false;

    private boolean manualRollControl = false;
    private double manualRollPercent = 0.0;

    public DifferentialWrist(DifferentialWristIO io) {
        this.io = io;

        pitchController = new ProfiledPIDController(
            20,
            0,
            0,
            new TrapezoidProfile.Constraints(8, 8)
        );
        pitchController.setTolerance(0.3);

        rollController = new ProfiledPIDController(
            15,
            0,
            0,
            new TrapezoidProfile.Constraints(10, 12)
        );
        rollController.setTolerance(0.2);

        pitchFeedforward = new ArmFeedforward(0.05, 0.0, 0.8);

        setGoals(0.0, 0.0);
    }

    public void setActive() {
        setPitchActive(true);
        setRollActive(true);
    }

    public void setPitchActive(boolean active) {
        pitchActive = active;
    }

    public void setRollActive(boolean active) {
        rollActive = active;
    }

    public void setGoals(double pitchRadians, double rollRadians) {
        pitchController.setGoal(pitchRadians);
        rollController.setGoal(rollRadians);
        Logger.recordOutput("DifferentialWrist/PitchGoal", pitchRadians);
        Logger.recordOutput("DifferentialWrist/RollGoal", rollRadians);
    }

    public void setPitchGoal(double pitchRadians) {
        pitchController.setGoal(pitchRadians);
        Logger.recordOutput("DifferentialWrist/PitchGoal", pitchRadians);
    }

    public void setRollGoal(double rollRadians) {
        rollController.setGoal(rollRadians);
        Logger.recordOutput("DifferentialWrist/RollGoal", rollRadians);
    }

    public double getPitchPosition() {
        return inputs.pitchPositionRadians;
    }

    public double getRollPosition() {
        return inputs.rollPositionRadians;
    }

    public double getPitchSetpointPosition() {
        return pitchController.getSetpoint().position;
    }

    public double getRollSetpointPosition() {
        return rollController.getSetpoint().position;
    }

    public double getPitchGoalPosition() {
        return pitchController.getGoal().position;
    }

    public double getRollGoalPosition() {
        return rollController.getGoal().position;
    }

    public void setAutoHome(boolean request) {
        autoHomeRequested = request;
        if (!request) {
            isHomed = false;
        }
    }

    public void autoHome() {
        autoHomeRequested = true;
        io.runPitchOpenLoop(0.04);

        Logger.recordOutput("DifferentialWrist/HomingPitchVelocity", inputs.pitchVelocityRadiansPerSecond);
        Logger.recordOutput("DifferentialWrist/LeftCurrent", inputs.leftCurrentAmps);
        Logger.recordOutput("DifferentialWrist/RightCurrent", inputs.rightCurrentAmps);

        if (Math.abs(inputs.pitchVelocityRadiansPerSecond) <= 0.05) {
            io.resetPitchPosition(0.0);
            io.stop();
            setHomed();
        }
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setHomed() {
        io.resetPitchPosition(0.0);
        io.resetRollPosition(0.0);
        isHomed = true;
        autoHomeRequested = false;
    }

    public void stop() {
        io.stop();
    }

    public boolean isPitchAtGoal() {
        return Math.abs(pitchController.getPositionError()) < 0.05
            && pitchController.getSetpoint().position == pitchController.getGoal().position;
    }

    public boolean isRollAtGoal() {
        return Math.abs(rollController.getPositionError()) < 0.02
            && rollController.getSetpoint().position == rollController.getGoal().position;
    }

    public boolean atGoal() {
        return isPitchAtGoal() && isRollAtGoal();
    }

    public void controlRollManually(double percent) {
        manualRollControl = true;
        manualRollPercent = percent;
    }

    public void disableManualControl() {
        manualRollControl = false;
        manualRollPercent = 0.0;
    }

    private void applyClosedLoopOutputs() {
        double pitchMeasurement = getPitchPosition();
        double rollMeasurement = getRollPosition();

        double leftVoltage = 0.0;
        double rightVoltage = 0.0;

        if (pitchActive) {
            double pitchOutput = pitchController.calculate(pitchMeasurement);
            TrapezoidProfile.State pitchSetpoint = pitchController.getSetpoint();
            double pitchFeedforwardVoltage = pitchFeedforward.calculate(
                pitchSetpoint.position,
                pitchSetpoint.velocity
            );
            double pitchVoltage = pitchOutput + pitchFeedforwardVoltage;
            leftVoltage += pitchVoltage;
            rightVoltage += pitchVoltage;
            Logger.recordOutput("DifferentialWrist/PitchSetpointPosition", pitchSetpoint.position);
            Logger.recordOutput("DifferentialWrist/PitchSetpointVelocity", pitchSetpoint.velocity);
            Logger.recordOutput("DifferentialWrist/PitchError", pitchController.getPositionError());
            Logger.recordOutput("DifferentialWrist/PitchVoltage", pitchVoltage);
            Logger.recordOutput("DifferentialWrist/PitchPIDVoltage", pitchOutput);
            Logger.recordOutput("DifferentialWrist/PitchFeedforwardVolts", pitchFeedforwardVoltage);
        }

        if (rollActive) {
            double rollOutput = rollController.calculate(rollMeasurement);
            leftVoltage += rollOutput;
            rightVoltage -= rollOutput;
            Logger.recordOutput("DifferentialWrist/RollSetpointPosition", rollController.getSetpoint().position);
            Logger.recordOutput("DifferentialWrist/RollSetpointVelocity", rollController.getSetpoint().velocity);
            Logger.recordOutput("DifferentialWrist/RollError", rollController.getPositionError());
            Logger.recordOutput("DifferentialWrist/RollVoltage", rollOutput);
        }

        io.setMotorVoltages(leftVoltage, rightVoltage);
        Logger.recordOutput("DifferentialWrist/LeftAppliedVolts", leftVoltage);
        Logger.recordOutput("DifferentialWrist/RightAppliedVolts", rightVoltage);
    }

    private void applyManualOutputs() {
        double rollVoltage = manualRollPercent * 12.0;
        double leftVoltage = rollVoltage;
        double rightVoltage = -rollVoltage;
        io.setMotorVoltages(leftVoltage, rightVoltage);
        Logger.recordOutput("DifferentialWrist/ManualRollPercent", manualRollPercent);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("DifferentialWrist", inputs);

        Logger.recordOutput("DifferentialWrist/PitchMeasurement", getPitchPosition());
        Logger.recordOutput("DifferentialWrist/RollMeasurement", getRollPosition());

        if (autoHomeRequested && !isHomed) {
            autoHome();
        } else if (manualRollControl) {
            applyManualOutputs();
        } else {
            applyClosedLoopOutputs();
        }

        Logger.recordOutput("DifferentialWrist/PitchGoal", pitchController.getGoal().position);
        Logger.recordOutput("DifferentialWrist/PitchPosition", getPitchPosition());
        Logger.recordOutput("DifferentialWrist/RollGoal", rollController.getGoal().position);
        Logger.recordOutput("DifferentialWrist/RollPosition", getRollPosition());
        Logger.recordOutput("DifferentialWrist/ManualMode", manualRollControl);
        Logger.recordOutput("DifferentialWrist/PitchActive", pitchActive);
        Logger.recordOutput("DifferentialWrist/RollActive", rollActive);
    }
}
