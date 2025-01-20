package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Grabber2D;
import frc.robot.subsystems.Elevator;
import frc.robot.StateManager;
import frc.robot.subsystems.Arm;

public class Stow extends Command{

    private Grabber2D util_Grabber2d;
    
    private Elevator s_Elevator;
    private Arm s_Arm;

    public Stow(Elevator s_Elevator, Arm s_Arm) {

        this.s_Elevator = s_Elevator;
        this.s_Arm = s_Arm;

        addRequirements(s_Elevator, s_Arm);

    }

    @Override
    public void execute() {
        util_Grabber2d.moveToWithFixedAngle(StateManager.getCoordinate(StateManager.Positions.STOW)[0], StateManager.getCoordinate(StateManager.Positions.STOW)[1]); //TODO this might not work

    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
