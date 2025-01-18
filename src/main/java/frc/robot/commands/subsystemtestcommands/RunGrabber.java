package frc.robot.commands.subsystemtestcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Grabber;

public class RunGrabber extends Command{

    private Grabber s_Grabber;
    
    public RunGrabber(Grabber s_Grabber) {

        this.s_Grabber = s_Grabber;

        addRequirements(s_Grabber);
    }


    @Override
    public void execute() {

        s_Grabber.RunGrabber();
    }

    @Override
    public void end(boolean interrupted) {
        s_Grabber.stopGrabber();
    }
}
