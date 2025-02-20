package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.subsystemController;
import frc.robot.subsystems.Swerve;
import frc.robot.StateManager;
import frc.robot.subsystems.Hexatroller;
import frc.lib.util.OnTheFlyTrajectory;

public class scoringAutoAlign extends Command{

    private Swerve s_Swerve;

    private OnTheFlyTrajectory newOnTheFlyPath = new OnTheFlyTrajectory(s_Swerve);

    public scoringAutoAlign(Swerve s_Swerve, OnTheFlyTrajectory newOnTheFlyTrajectory) {

        this.s_Swerve = s_Swerve;
        this.newOnTheFlyPath = newOnTheFlyTrajectory;

        addRequirements(s_Swerve);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_Swerve.followPathCommand(newOnTheFlyPath.newOnTheFlyPath());
    }


    @Override
    public void end(boolean interrupted) {

    }
}
