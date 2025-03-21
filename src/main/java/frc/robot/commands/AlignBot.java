package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.OnTheFlyTrajectory;
import frc.lib.util.graph.GraphParser;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.AlignTool;

public class AlignBot extends Command {
    private Swerve s_Swerve;
    private AlignTool AlignTool;

    private Pose2d whereToAlign;

    private OnTheFlyTrajectory onTheFlyTrajectory;

    public AlignBot(Swerve s_Swerve, AlignTool AlignTool, OnTheFlyTrajectory onTheFlyTrajectory) {
        this.s_Swerve = s_Swerve;
        this.AlignTool = AlignTool;
        this.onTheFlyTrajectory = onTheFlyTrajectory;

        addRequirements(s_Swerve, AlignTool);
    }

    @Override
    public void initialize() {

        char requestedBranch = s_Swerve.findClosestBranch(
            DriverStation.getAlliance().orElse(null) == Alliance.Red ? 
            true : false
        );

        Pose2d whereToAlign = AlignTool.AlignTo(requestedBranch);


    }

    @Override
    public void execute() {

        s_Swerve.followPathCommand(onTheFlyTrajectory.newOnTheFlyPath(whereToAlign));
    }

    @Override
    public void end(boolean interrupted) {

    }
}
