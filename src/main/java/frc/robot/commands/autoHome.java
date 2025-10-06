package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.GraphParser.GraphData;
import frc.lib.util.graph.Node;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;

public class autoHome extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;

    private Pivot s_Pivot;
    private Elevator s_Elevator;
    private WristPitch s_WristPitch;
    private WristRoll s_WristRoll;

    public autoHome(SubsystemManager manager, Pivot s_Pivot, 
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
        System.err.println("Hello AutoHome command");
    }

    @Override
    public void execute() {
        if (manager.canAutoHome()) {
            s_Pivot.autoHome();
            s_Elevator.autoHome();
            s_WristPitch.autoHome();
            System.err.println("AutoHoming");
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
