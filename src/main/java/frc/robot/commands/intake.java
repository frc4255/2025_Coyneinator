package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Grabber2D;
import frc.robot.StateManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.wrist.EndEffector;
import frc.robot.subsystems.elevator.Elevator;

public class intake extends Command{

    private Grabber2D util_Grabber2d;
    
    private Elevator s_Elevator;
    private Arm s_Arm; 
    private EndEffector s_EndEffector;

    public intake(Elevator s_Elevator, Arm s_Arm, EndEffector s_EndEffector) {

        this.s_Elevator = s_Elevator;
        this.s_Arm = s_Arm;
        this.s_EndEffector = s_EndEffector;

        addRequirements(s_Elevator, s_Arm, s_EndEffector);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        util_Grabber2d.moveToWithFixedAngle(StateManager.getCoordinate(StateManager.Positions.HP)[0], StateManager.getCoordinate(StateManager.Positions.HP)[1]);

        s_EndEffector.runEndEffector();

    }


    @Override
    public void end(boolean interrupted) {
        util_Grabber2d.moveToWithFixedAngle(StateManager.getCoordinate(StateManager.Positions.STOW)[0], StateManager.getCoordinate(StateManager.Positions.STOW)[1]);
    }
}
