package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.subsystemController;
import frc.robot.StateManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.wrist.EndEffector;
import frc.robot.subsystems.wrist.WristManager;
import frc.robot.subsystems.elevator.Elevator;

public class driverManualScoring extends Command{
    
    private Elevator s_Elevator;
    private WristManager s_Wrist;
    private StateManager.Positions position;
    private subsystemController subsystemController;

    public driverManualScoring(Elevator s_Elevator, WristManager s_Wrist, StateManager.Positions position, subsystemController subsystemController) {

        this.s_Elevator = s_Elevator;
        this.s_Wrist = s_Wrist;
        this.position = position;
        this.subsystemController = subsystemController;

        addRequirements(s_Elevator, s_Wrist);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        subsystemController.BotShouldGoTo(StateManager.getCoordinate(position)[0], StateManager.getCoordinate(position)[1], StateManager.getCoordinate(position)[2], StateManager.getCoordinate(position)[3]);

    }


    @Override
    public void end(boolean interrupted) {

    }
}
