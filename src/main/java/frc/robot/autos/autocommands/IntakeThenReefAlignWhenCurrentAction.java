package frc.robot.autos.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;

public class IntakeThenReefAlignWhenCurrentAction extends Command {

    private SubsystemManager manager;
    private EndEffector endEffector;

    private Pivot s_Pivot;
    private Elevator s_Elevator;
    private WristPitch s_WristPitch;
    private WristRoll s_WristRoll;
    
    public IntakeThenReefAlignWhenCurrentAction(SubsystemManager manager, EndEffector endEffector, Pivot s_Pivot, 
        Elevator s_Elevator, WristPitch s_WristPitch, WristRoll s_WristRoll) {

        this.manager = manager;
        this.endEffector = endEffector;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_WristPitch = s_WristPitch;
        this.s_WristRoll = s_WristRoll;

        addRequirements(s_Pivot, s_Elevator, s_WristPitch, s_WristRoll);
    }

    @Override
    public void initialize() {
        manager.requestNode(GraphParser.getNodeByName("Coral Ground Pickup"));
        endEffector.setDutyCycle(-8);
    }

    @Override
    public void execute() {
        if (endEffector.getMotorCurrent() > 19) {
            endEffector.setDutyCycle(-0.5);
            manager.requestNode(GraphParser.getNodeByName("Reef Align"));
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
