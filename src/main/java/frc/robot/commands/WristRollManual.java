package frc.robot.commands;

import frc.robot.subsystems.DifferentialWrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class WristRollManual extends Command {
    private final DifferentialWrist s_Wrist;
    private final DoubleSupplier operatorHorizontalAxis;

    public WristRollManual(DifferentialWrist s_Wrist, DoubleSupplier operatorHorizontalAxis) {
        this.s_Wrist = s_Wrist;
        this.operatorHorizontalAxis = operatorHorizontalAxis;
        addRequirements(s_Wrist);
    }

    @Override
    public void execute() {
        double appliedValue = MathUtil.applyDeadband(operatorHorizontalAxis.getAsDouble(), 0.05);
        s_Wrist.controlRollManually(appliedValue);
    }

    @Override
    public void end(boolean interrupted) {
        s_Wrist.disableManualControl();
    }
}
