package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class ExtakeAlgae extends Command {

    private EndEffector endEffector;
    
    public ExtakeAlgae(EndEffector endEffector) {
        this.endEffector = endEffector;

        addRequirements(endEffector);
    }

     @Override
    public void initialize() {
        endEffector.setDutyCycle(Constants.EndEffector.OUTTAKE_ALGAE_VOLTS);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.stop();
    }
}
