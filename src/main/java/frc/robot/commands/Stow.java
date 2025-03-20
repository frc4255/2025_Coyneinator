package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.GraphParser.GraphData;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

public class Stow extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;

    public Stow(SubsystemManager manager, Pivot s_Pivot, Elevator s_Elevator) {
        this.manager = manager;
        
        addRequirements();
    }

    @Override
    public void initialize() {
        manager.requestNode(GraphParser.getNodeByName("Stow"));
    }
}
