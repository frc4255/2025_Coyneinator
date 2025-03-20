package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.OnTheFlyTrajectory;
import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.GraphParser.GraphData;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.AlignTool;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision.Camera;

public class L1Assist extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;

    private Swerve s_Swerve;
    private AlignTool AlignTool;

    private Pose2d whereToAlign;

    private OnTheFlyTrajectory onTheFlyTrajectory;

    private List<Camera> cams;
    
    private char sector;

    public L1Assist(SubsystemManager manager, Swerve swerve, AlignTool AlignTool, OnTheFlyTrajectory onTheFlyTrajectory) {
        this.manager = manager;
        this.s_Swerve = swerve;
        this.AlignTool = AlignTool;
        this.onTheFlyTrajectory = onTheFlyTrajectory;
        
        addRequirements(s_Swerve, AlignTool);
    }

    @Override
    public void initialize() {
        //TODO: Insert try catch here bc we don't want stupid code to make us die
        //dector = s_Swerve.getCurrentBranchSector(DriverStation.getAlliance().get() == Alliance.Red ? true : false);

        char requestedBranch = s_Swerve.findClosestBranch(
            DriverStation.getAlliance().orElse(null) == Alliance.Red ? 
            true : false
        );

        Pose2d whereToAlign = AlignTool.AlignTo(requestedBranch);

        s_Swerve.followPathCommand(onTheFlyTrajectory.newOnTheFlyPath(whereToAlign));
    }

    @Override
    public void execute() {

        manager.requestNode(GraphParser.getNodeByName("L1 Score"));
    }
}
