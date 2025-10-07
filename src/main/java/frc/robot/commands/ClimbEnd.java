package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.WristPitch;

public class ClimbEnd extends Command {
    
    private SubsystemManager manager;

    private Pivot s_Pivot;
    private Climber s_Climber;
    private WristPitch s_WristPitch;

    public ClimbEnd(SubsystemManager manager, Climber s_Climber, Pivot s_Pivot, WristPitch s_WristPitch) {

        this.manager = manager;

        this.s_Climber = s_Climber;
        this.s_WristPitch = s_WristPitch;
        this.s_Pivot = s_Pivot;

        addRequirements(s_Pivot, s_Climber, s_WristPitch);
    }

    @Override
    public void initialize() {
        s_Climber.setVoltage(10);


    }

    @Override
    public void execute() {

        s_Pivot.setVoltageForClimb();

        if (s_Climber.getMotorCurrent() > 15) {
            s_Climber.setVoltage(8);

        }

        if (s_Pivot.velocityOfMotors() <= 0.05 && s_Pivot.getPivotPosition() < 0.05) {
            s_Pivot.stopMotors();
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
