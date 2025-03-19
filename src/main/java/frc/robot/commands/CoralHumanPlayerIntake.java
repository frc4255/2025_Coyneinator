package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.EndEffector;

public class CoralHumanPlayerIntake extends Command {
    
    private SubsystemManager manager;
    private EndEffector endEffector;

    public CoralHumanPlayerIntake(SubsystemManager manager, EndEffector endEffector) {
        this.manager = manager;
        this.endEffector = endEffector;
    }

    @Override
    public void initialize() {
        manager.requestNode(GraphParser.getNodeByName("Coral HP Pickup"));
        endEffector.setDutyCycle(-0.5);


    }

    @Override
    public void execute() {
        if (endEffector.getMotorCurrent() > 10) {
            endEffector.setDutyCycle(-0.05);

        }
    }

    @Override
    public void end(boolean interrupted) {
        manager.requestNode(GraphParser.getNodeByName("Stow"));
    }
}
