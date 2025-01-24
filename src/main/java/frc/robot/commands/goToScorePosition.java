package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Grabber2D;
import frc.robot.StateManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hexatroller;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.Grabber;

public class goToScorePosition extends Command{

    private Grabber2D util_Grabber2d;
    
    private Elevator s_Elevator;
    private Arm s_Arm;
    private Hexatroller s_Hexatroller; 
    private Grabber s_Grabber;

    public goToScorePosition(Elevator s_Elevator, Arm s_Arm, Hexatroller s_Hexatroller, Grabber s_Grabber) {

        this.s_Elevator = s_Elevator;
        this.s_Arm = s_Arm;
        this.s_Hexatroller = s_Hexatroller;
        this.s_Grabber = s_Grabber;

        addRequirements(s_Elevator, s_Arm, s_Hexatroller, s_Grabber);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double[] whereToScore = s_Hexatroller.whereToScore();

        util_Grabber2d.moveToWithFixedAngle(whereToScore[0], whereToScore[1]);

    }


    @Override
    public void end(boolean interrupted) {

        util_Grabber2d.moveToWithFixedAngle(StateManager.getCoordinate(StateManager.Positions.STOW)[0], StateManager.getCoordinate(StateManager.Positions.STOW)[1]); //TODO this might not work

    }
}
