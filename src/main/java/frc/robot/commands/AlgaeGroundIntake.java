package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.DifferentialWrist;

public class AlgaeGroundIntake extends Command {
    
    private SubsystemManager manager;
    private EndEffector endEffector;

    private Pivot s_Pivot;
    private Elevator s_Elevator;
    private DifferentialWrist s_Wrist;
    public AlgaeGroundIntake(SubsystemManager manager, EndEffector endEffector, Pivot s_Pivot, 
        Elevator s_Elevator, DifferentialWrist s_Wrist) {

        this.manager = manager;
        this.endEffector = endEffector;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_Wrist = s_Wrist;
        addRequirements(s_Pivot, s_Elevator, s_Wrist);
    }

    @Override
    public void initialize() {
        manager.requestNode(GraphParser.getNodeByName("Algae Ground Pickup"));
        endEffector.setDutyCycle(12);


    }

    @Override
    public void execute() {
        if (endEffector.getMotorCurrent() > 25) {
            endEffector.setDutyCycle(10);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
