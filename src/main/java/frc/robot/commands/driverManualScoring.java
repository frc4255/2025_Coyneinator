package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Grabber2D;
import frc.robot.StateManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.elevator.Elevator;

public class driverManualScoring extends Command{
    
    private Elevator s_Elevator;
    private Arm s_Arm; 
    private StateManager.Positions position;
    private Grabber2D grabber2D;

    public driverManualScoring(Elevator s_Elevator, Arm s_Arm, StateManager.Positions position, Grabber2D grabber2d) {

        this.s_Elevator = s_Elevator;
        this.s_Arm = s_Arm;
        this.position = position;
        this.grabber2D = grabber2d;

        addRequirements(s_Elevator, s_Arm);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        grabber2D.moveToWithFixedAngle(StateManager.getCoordinate(position)[0], StateManager.getCoordinate(position)[1]);

    }


    @Override
    public void end(boolean interrupted) {

    }
}
