package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.WristPitchIOInputsAutoLogged;

public class WristPitch extends SubsystemBase {
    private final WristPitchIO io;
    private final WristPitchIOInputsAutoLogged inputs = new WristPitchIOInputsAutoLogged();

    private final ProfiledPIDController controller;
    private final ArmFeedforward feedforward;

    private boolean isHomed = false;
    private boolean isPosePossible = true;
    private boolean active = false;

    public WristPitch(WristPitchIO io) {
        this.io = io;

        controller = new ProfiledPIDController(
            20,
            0,
            0,
            new TrapezoidProfile.Constraints(8, 8)
        );
        controller.setTolerance(0.3);

        feedforward = new ArmFeedforward(0.05, 0.0, 0.8);

        setGoal(0.0);
    }

    public void setActive() {
        active = true;
    }

    public void setAutoHome(boolean request) {
        isHomed = request;
    }

    public void autoHome() {
        io.runOpenLoop(0.04);

        Logger.recordOutput("WristPitch/HomingVelocity", inputs.velocityRadiansPerSecond);
        Logger.recordOutput("WristPitch/HomingCurrent", inputs.currentAmps);

        if (Math.abs(inputs.velocityRadiansPerSecond) <= 0.05) {
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

    public void setGoal(double pos) {
        controller.setGoal(pos);
    }

    public double getCurrentPos() {
        return inputs.positionRadians;
    }

    public void setHomed() {
        io.resetPosition(0.0);
    }

    public boolean isPivotPosePossible() {
        return isPosePossible;
    }

    public void stopMotors() {
        io.stop();
    }

    public boolean atGoal() {
        return Math.abs(controller.getPositionError()) < 0.05
            && controller.getSetpoint().position == controller.getGoal().position;
    }

    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        double finalOut = output + feedforward.calculate(setpoint.position, setpoint.velocity);
        io.setVoltage(finalOut);
        Logger.recordOutput("WristPitch/AppliedVoltage", finalOut);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("WristPitch", inputs);

        if (active) {
            double measurement = getCurrentPos();
            double pidOutput = controller.calculate(measurement);
            useOutput(pidOutput, controller.getSetpoint());
        }

        Logger.recordOutput("WristPitch/Goal", controller.getGoal().position);
        Logger.recordOutput("WristPitch/Position", getCurrentPos());
        Logger.recordOutput("WristPitch/Velocity", inputs.velocityRadiansPerSecond);
        Logger.recordOutput("WristPitch/Acceleration", inputs.accelerationRadiansPerSecondSq);
    }
}
