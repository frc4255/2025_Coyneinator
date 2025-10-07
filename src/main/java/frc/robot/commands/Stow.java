package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;

public class Stow extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;

    private Pivot s_Pivot;
    private Elevator s_Elevator;
    private WristPitch s_WristPitch;
    private WristRoll s_WristRoll;

    private boolean stopHoming = false;

    public Stow(SubsystemManager manager, Pivot s_Pivot, 
        Elevator s_Elevator, WristPitch s_WristPitch, WristRoll s_WristRoll) {
        this.manager = manager;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_WristPitch = s_WristPitch;
        this.s_WristRoll = s_WristRoll;
        
        addRequirements(s_Pivot, s_Elevator, s_WristPitch, s_WristRoll);
    }

    @Override
    public void initialize() {
        manager.requestNode("Stow");

        /*
        s_Elevator.setAutoHome(false);
        s_Pivot.setAutoHome(false);
        s_WristPitch.setAutoHome(false);
        */

    }

    @Override
    public void execute() {

        /*
        System.err.println("stop homing is: " + stopHoming);
        if (manager.canAutoHome() && !stopHoming) {
            System.err.println("AutoHoming");
            s_Pivot.autoHome();
            s_Elevator.autoHome();
            s_WristPitch.autoHome();
        }

        if (s_Elevator.isHomed() && s_Pivot.isHomed() && s_WristPitch.isHomed()) {
            stopHoming = true;
        } 
             
        */
    }

    @Override
    public void end(boolean interrupted) {

    }
}
