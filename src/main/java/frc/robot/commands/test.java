package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.GraphParser.GraphData;
import frc.robot.SubsystemManager;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;

public class test extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;
    private Elevator s_Swerve;

    public test(Elevator swerve) {
        this.s_Swerve = swerve;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        s_Swerve.setGoal(.5);
    }
}
