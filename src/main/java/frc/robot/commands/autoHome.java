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

public class autoHome extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;

    private Pivot s_Pivot;
    private Elevator s_Elevator;
    private DifferentialWrist s_Wrist;
    public autoHome(SubsystemManager manager, Pivot s_Pivot, 
        Elevator s_Elevator, DifferentialWrist s_Wrist) {
        this.manager = manager;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_Wrist = s_Wrist;
        addRequirements(s_Pivot, s_Elevator, s_Wrist);
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
            s_Wrist.autoHome();
            System.err.println("AutoHoming");
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
