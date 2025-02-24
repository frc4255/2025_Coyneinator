package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.subsystemController;
import frc.robot.Constants.Grabber;
import frc.robot.StateManager;
import frc.robot.subsystems.Hexatroller;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.wrist.EndEffector;
import frc.robot.subsystems.wrist.WristManager;

public class goToScorePosition extends Command{

    private subsystemController subsystemController;
    
    private Elevator s_Elevator;
    private Pivot s_Pivot;
    private WristManager s_Wrist;
    private Hexatroller s_Hexatroller; 
    private EndEffector s_EndEffector;

    private double[] stowPosition;

    public goToScorePosition(Elevator s_Elevator, Pivot s_Pivot, WristManager s_Wrist, Hexatroller s_Hexatroller, EndEffector s_EndEffector) {

        this.s_Elevator = s_Elevator;
        this.s_Pivot = s_Pivot;
        this.s_Wrist = s_Wrist;
        this.s_Hexatroller = s_Hexatroller;
        this.s_EndEffector = s_EndEffector;

        addRequirements(s_Elevator, s_Pivot, s_Wrist, s_Hexatroller, s_EndEffector);

    }

    @Override
    public void initialize() {
        stowPosition = StateManager.getCoordinate(StateManager.Positions.STOW);
    }

    @Override
    public void execute() {
        double[] whereToScore = s_Hexatroller.whereToScore();

        subsystemController.BotShouldGoTo(whereToScore[0], whereToScore[1], whereToScore[2], whereToScore[3]);

    }


    @Override
    public void end(boolean interrupted) {

        subsystemController.BotShouldGoTo(stowPosition[0], stowPosition[1], stowPosition[2], stowPosition[3]);

    }
}
