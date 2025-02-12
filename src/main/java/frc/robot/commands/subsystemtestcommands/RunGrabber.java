package frc.robot.commands.subsystemtestcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Grabber;
import frc.robot.subsystems.wrist.EndEffector;

public class RunGrabber extends Command{

    private EndEffector s_EndEffector;
    
    public RunGrabber(EndEffector s_EndEffector) {

        this.s_EndEffector = s_EndEffector;

        addRequirements(s_EndEffector);
    }


    @Override
    public void execute() {

        s_EndEffector.runEndEffector();
    }

    @Override
    public void end(boolean interrupted) {
        s_EndEffector.stop();
    }
}
