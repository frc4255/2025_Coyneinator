package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.DifferentialWrist;

public class AlgaeL2Pickup extends Command {
    
    private SubsystemManager manager;
    private EndEffector endEffector;

    private Pivot s_Pivot;
    private Elevator s_Elevator;
    private DifferentialWrist s_Wrist;
    public AlgaeL2Pickup(SubsystemManager manager, EndEffector endEffector, Pivot s_Pivot, 
        Elevator s_Elevator, DifferentialWrist s_Wrist) {

        this.manager = manager;
        this.endEffector = endEffector;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_Wrist = s_Wrist;
        addRequirements(s_Pivot, endEffector, s_Elevator, s_Wrist);
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
