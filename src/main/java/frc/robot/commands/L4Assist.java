package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.GraphParser.GraphData;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.Swerve;

public class L4Assist extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;
    private Swerve s_Swerve;

    public L4Assist(SubsystemManager manager, Swerve swerve) {
        this.manager = manager;
        this.s_Swerve = swerve;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        manager.requestNode(GraphParser.getNodeByName("L4 Init"));
    }
}
