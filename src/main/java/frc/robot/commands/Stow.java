package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.subsystemController;
import frc.robot.StateManager;
import frc.robot.subsystems.wrist.WristManager;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Pivot;

public class Stow extends Command{
    
    private Elevator s_Elevator;
    private Pivot s_Pivot;
    private WristManager s_Wrist;
    private subsystemController subsystemController;

    private StateManager.Positions position;

    public Stow(Elevator s_Elevator, Pivot s_Pivot, WristManager s_Wrist, subsystemController subsystemController) {

        this.s_Elevator = s_Elevator;
        this.s_Pivot = s_Pivot;
        this.s_Wrist = s_Wrist;
        this.subsystemController = subsystemController;

        addRequirements(s_Elevator, s_Pivot, s_Wrist);

    }

    @Override
    public void initialize() {
        position = StateManager.Positions.STOW;
    }

    @Override
    public void execute() {

        subsystemController.BotShouldGoTo(StateManager.getCoordinate(position)[0], StateManager.getCoordinate(position)[1], StateManager.getCoordinate(position)[2], StateManager.getCoordinate(position)[3]);

    }


    @Override
    public void end(boolean interrupted) {

    }
}
