package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class ExtakeCoral extends Command{

    private final EndEffector s_EndEffector;
    
    public ExtakeCoral(EndEffector s_endEffector) {
        this.s_EndEffector = s_endEffector;

        addRequirements(s_endEffector);
    }

    @Override
    public void initialize() {
        s_EndEffector.setDutyCycle(2);
    }

    @Override
    public void end(boolean interrupted) {
        s_EndEffector.stop();
    }
}
