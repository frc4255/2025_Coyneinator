package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristRoll;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class WristRollManual extends Command {    
    private WristRoll s_WristRoll;   
    private DoubleSupplier operatorHorizontalAxis;

    public WristRollManual(WristRoll s_WristRoll, DoubleSupplier operatorHorizontalAxis) {
        this.s_WristRoll = s_WristRoll;
        addRequirements(s_WristRoll);

        this.operatorHorizontalAxis = operatorHorizontalAxis;
    }

    @Override
    public void execute() {
        
        double appliedValue = MathUtil.applyDeadband(operatorHorizontalAxis.getAsDouble(), 0.05);
        s_WristRoll.controlManually(appliedValue);
    }
}