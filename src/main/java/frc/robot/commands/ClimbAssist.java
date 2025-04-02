package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.robot.SubsystemManager;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;

public class ClimbAssist extends Command {
    
    private SubsystemManager manager;

    private Pivot s_Pivot;
    private Climber s_Climber;
    private WristPitch s_WristPitch;

    public ClimbAssist(SubsystemManager manager, Climber s_Climber, Pivot s_Pivot, WristPitch s_WristPitch) {

        this.manager = manager;

        this.s_Climber = s_Climber;
        this.s_WristPitch = s_WristPitch;
        this.s_Pivot = s_Pivot;

        addRequirements(s_Pivot, s_Climber, s_WristPitch);
    }

    @Override
    public void initialize() {
        manager.requestNode(GraphParser.getNodeByName("Climb"));
        s_Climber.setVoltage(10);


    }

    @Override
    public void execute() {
        if (s_Climber.getMotorCurrent() > 15) {
            s_Climber.setVoltage(6);

        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
