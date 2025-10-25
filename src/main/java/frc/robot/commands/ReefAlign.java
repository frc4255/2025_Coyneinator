package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.GraphParser.GraphData;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.AlignTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.DifferentialWrist;
import frc.robot.subsystems.Vision.Camera;

public class ReefAlign extends Command {

    private PIDController rotPidController;
    private ProfiledPIDController translationPidController;
    private SubsystemManager manager;

    private Swerve s_Swerve;
    private AlignTool AlignTool;

    private Pose2d whereToAlign;

    private List<Camera> cams;
    
    private char sector;

    private Pivot s_Pivot;
    private Elevator s_Elevator;
    private DifferentialWrist s_Wrist;
    public ReefAlign(SubsystemManager manager, Pivot s_Pivot, 
        Elevator s_Elevator, DifferentialWrist s_Wrist) {

        this.manager = manager;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_Wrist = s_Wrist;
        addRequirements(s_Pivot, s_Elevator, s_Wrist);
    }

    @Override
    public void initialize() {
        //TODO: Insert try catch here bc we don't want stupid code to make us die
        //dector = s_Swerve.getCurrentBranchSector(DriverStation.getAlliance().get() == Alliance.Red ? true : false);
        /*
        char requestedBranch = s_Swerve.findClosestBranch(
            DriverStation.getAlliance().orElse(null) == Alliance.Red ? 
            true : false
        );

        Pose2d whereToAlign = AlignTool.AlignTo(requestedBranch);

        s_Swerve.followPathCommand(onTheFlyTrajectory.newOnTheFlyPath(whereToAlign));
         */
        manager.requestNode(GraphParser.getNodeByName("Reef Align"));
        

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

}
