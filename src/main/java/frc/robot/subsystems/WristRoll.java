package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.WristRollIOInputsAutoLogged;

public class WristRoll extends SubsystemBase {
    private final WristRollIO io;
    private final WristRollIOInputsAutoLogged inputs = new WristRollIOInputsAutoLogged();

    private final ProfiledPIDController controller;

    private boolean isHomed = false;
    private boolean isPosePossible = true;
    private boolean active = false;

    public WristRoll(WristRollIO io) {
        this.io = io;

        controller = new ProfiledPIDController(
            15,
            0,
            0,
            new TrapezoidProfile.Constraints(10, 12)
        );
        controller.setTolerance(0.2);
    }

    public void controlManually(double request) {
        io.runOpenLoop(request);
    }

    public void setActive() {
        active = true;
    }

    public double getCurrentPos() {
        return inputs.positionRadians;
    }

    public void setHomed() {
        io.resetPosition(0.0);
        isHomed = true;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setGoal(double pos) {
        controller.setGoal(pos);
    }

    public boolean atGoal() {
        return Math.abs(controller.getPositionError()) < 0.02
            && controller.getSetpoint().position == controller.getGoal().position;
    }

    public boolean isPivotPosePossible() {
        return isPosePossible;
    }

    public void stopMotor() {
        io.stop();
    }

    private void applyOutput(double output) {
        io.setVoltage(output);
        Logger.recordOutput("WristRoll/AppliedVoltage", output);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("WristRoll", inputs);

        if (active) {
            double measurement = getCurrentPos();
            double pidOutput = controller.calculate(measurement);
            applyOutput(pidOutput);
        }

        Logger.recordOutput("WristRoll/Position", getCurrentPos());
    }
}
