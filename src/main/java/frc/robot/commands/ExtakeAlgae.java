package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.robot.subsystems.EndEffector;

public class ExtakeAlgae extends Command {

    private EndEffector endEffector;
    
    public ExtakeAlgae(EndEffector endEffector) {
        this.endEffector = endEffector;

        addRequirements(endEffector);
    }

     @Override
    public void initialize() {


    }

    @Override
    public void execute() {
        endEffector.setDutyCycle(-8);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
