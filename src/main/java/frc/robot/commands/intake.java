package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.subsystemController;
import frc.robot.StateManager;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.wrist.EndEffector;
import frc.robot.subsystems.wrist.WristManager;
import frc.robot.subsystems.elevator.Elevator;

public class intake extends Command{

    private subsystemController subsystemController;
    
    private Elevator s_Elevator;
    private Pivot s_Pivot;
    private WristManager s_Wrist;
    private EndEffector s_EndEffector;

    public intake(Elevator s_Elevator, Pivot s_Pivot, WristManager s_Wrist, EndEffector s_EndEffector) {

        this.s_Elevator = s_Elevator;
        this.s_Pivot = s_Pivot;
        this.s_Wrist = s_Wrist;
        this.s_EndEffector = s_EndEffector;

        addRequirements(s_Elevator, s_Pivot, s_Wrist, s_EndEffector);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        subsystemController.BotShouldGoTo(StateManager.getCoordinate(StateManager.Positions.HP)[0], StateManager.getCoordinate(StateManager.Positions.HP)[1], 
                                StateManager.getCoordinate(StateManager.Positions.HP)[2], StateManager.getCoordinate(StateManager.Positions.HP)[3]);

        s_EndEffector.runEndEffector();

    }


    @Override
    public void end(boolean interrupted) {
        subsystemController.BotShouldGoTo(StateManager.getCoordinate(StateManager.Positions.HP)[0], StateManager.getCoordinate(StateManager.Positions.HP)[1], 
                                StateManager.getCoordinate(StateManager.Positions.HP)[2], StateManager.getCoordinate(StateManager.Positions.HP)[3]);

        s_EndEffector.idleEndEffector();
    }
}
