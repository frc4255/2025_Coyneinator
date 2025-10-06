package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;

public class AlgaeL2Pickup extends Command {
    
    private SubsystemManager manager;
    private EndEffector endEffector;

    private Pivot s_Pivot;
    private Elevator s_Elevator;
    private WristPitch s_WristPitch;
    private WristRoll s_WristRoll;

    public AlgaeL2Pickup(SubsystemManager manager, EndEffector endEffector, Pivot s_Pivot, 
        Elevator s_Elevator, WristPitch s_WristPitch, WristRoll s_WristRoll) {

        this.manager = manager;
        this.endEffector = endEffector;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_WristPitch = s_WristPitch;
        this.s_WristRoll = s_WristRoll;

        addRequirements(s_Pivot, endEffector, s_Elevator, s_WristPitch, s_WristRoll);
    }

    @Override
    public void initialize() {
        manager.requestNode(GraphParser.getNodeByName("L2 Algae Pickup"));
        endEffector.setDutyCycle(10);


    }

    @Override
    public void execute() {
        if (endEffector.getMotorCurrent() > 30) {
            endEffector.setDutyCycle(8);

        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
