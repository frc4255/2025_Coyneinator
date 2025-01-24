package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Grabber2D;
import frc.robot.StateManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.elevator.Elevator;

public class intake extends Command{

    private Grabber2D util_Grabber2d;
    
    private Elevator s_Elevator;
    private Arm s_Arm; 
    private Grabber s_Grabber;

    public intake(Elevator s_Elevator, Arm s_Arm, Grabber s_Grabber) {

        this.s_Elevator = s_Elevator;
        this.s_Arm = s_Arm;
        this.s_Grabber = s_Grabber;

        addRequirements(s_Elevator, s_Arm, s_Grabber);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        util_Grabber2d.moveToWithFixedAngle(StateManager.getCoordinate(StateManager.Positions.HP)[0], StateManager.getCoordinate(StateManager.Positions.HP)[1]);

        s_Grabber.RunGrabber();

    }


    @Override
    public void end(boolean interrupted) {
        util_Grabber2d.moveToWithFixedAngle(StateManager.getCoordinate(StateManager.Positions.STOW)[0], StateManager.getCoordinate(StateManager.Positions.STOW)[1]);
    }
}
