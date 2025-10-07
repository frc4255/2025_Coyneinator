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
import frc.robot.subsystems.DifferentialWrist;

public class Stow extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;

    private Pivot s_Pivot;
    private Elevator s_Elevator;
    private DifferentialWrist s_Wrist;
    private boolean stopHoming = false;

    public Stow(SubsystemManager manager, Pivot s_Pivot, 
        Elevator s_Elevator, DifferentialWrist s_Wrist) {
        this.manager = manager;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_Wrist = s_Wrist;
        addRequirements(s_Pivot, s_Elevator, s_Wrist);
    }

    @Override
    public void initialize() {
        manager.requestNode(GraphParser.getNodeByName("Stow"));

        s_Elevator.setAutoHome(false);
        s_Pivot.setAutoHome(false);
        s_Wrist.setAutoHome(false);

    }

    @Override
    public void execute() {

        /*
        System.err.println("stop homing is: " + stopHoming);
        if (manager.canAutoHome() && !stopHoming) {
            System.err.println("AutoHoming");
            s_Pivot.autoHome();
            s_Elevator.autoHome();
            s_Wrist.autoHome();
        }

        if (s_Elevator.isHomed() && s_Pivot.isHomed() && s_Wrist.isHomed()) {
            stopHoming = true;
        } */
    }

    @Override
    public void end(boolean interrupted) {

    }
}
