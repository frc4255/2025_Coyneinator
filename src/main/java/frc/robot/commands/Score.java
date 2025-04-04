package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.GraphParser.GraphData;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.AlignTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;
import frc.robot.subsystems.Vision.Camera;

public class Score extends Command {

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
    private WristPitch s_WristPitch;
    private WristRoll s_WristRoll;

    //Relative to apriltag
    private Pose2d leftScoringPose; //odd
    private Pose2d rightScoringPose; //even
    private int level;

    private boolean canEnd;

    private final PIDController xController;
    private final  PIDController yController;
    private final PIDController headingController;

    private Pose2d activeTargetPose;

    public Score(int level, SubsystemManager manager, Pivot s_Pivot, 
        Elevator s_Elevator, WristPitch s_WristPitch, WristRoll s_WristRoll) {

        this.manager = manager;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_WristPitch = s_WristPitch;
        this.s_WristRoll = s_WristRoll;

        this.level = level;

        leftScoringPose = new Pose2d(new Translation2d(-0.2,0.6), new Rotation2d());
        rightScoringPose = new Pose2d(new Translation2d(0.2,0.6), new Rotation2d());

        xController = new PIDController(5.0, 0.0, 0.0);
        yController = new PIDController(5.0, 0.0, 0.0);
        headingController = new PIDController(2.5, 0.0, 0.0);

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        headingController.setTolerance(0.1);


        addRequirements(s_Pivot, s_Elevator, s_WristPitch, s_WristRoll);
    }

    @Override
    public void initialize() {
        manager.requestNode(GraphParser.getNodeByName("Reef Align"));

        char branch = s_Swerve.findClosestBranch(DriverStation.getAlliance().get() == Alliance.Red ? true : false);
        int branchNumber = frc.robot.FieldLayout.Reef.branchesToInt.get(branch);
        activeTargetPose = (branchNumber % 2 == 1) ? leftScoringPose : rightScoringPose;
    }

    @Override
    public void execute() {
        // Get the current robot pose from s_Swerve
        Pose2d currentPose = s_Swerve.getPose();
        
        // Transform activeTargetPose from apriltag-relative coordinates to field coordinates
        int tagID = FieldLayout.Reef.branchesToAprilTagID.get(sector);
        Pose3d tagPose3d = FieldLayout.AprilTags.APRIL_TAG_POSE.stream()
            .filter(tag -> tag.ID == tagID)
            .findFirst()
            .orElseThrow()
            .pose;
        Pose2d tagPose2d = new Pose2d(tagPose3d.getX(), tagPose3d.getY(), new Rotation2d(tagPose3d.getRotation().getZ()));
        Pose2d fieldTargetPose = tagPose2d.relativeTo(activeTargetPose);
        
        // Calculate PID outputs for x, y, and heading to move towards the field target pose
        double xCommand = xController.calculate(currentPose.getTranslation().getX(), fieldTargetPose.getTranslation().getX());
        double yCommand = yController.calculate(currentPose.getTranslation().getY(), fieldTargetPose.getTranslation().getY());
        double headingCommand = headingController.calculate(currentPose.getRotation().getRadians(), fieldTargetPose.getRotation().getRadians());
        
        // Command the swerve drive to move towards the target
        s_Swerve.drive(new Translation2d(xCommand, yCommand), headingCommand, false, false);

        if (xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint()) {
            manager.requestNode(GraphParser.getNodeByName("L" + level + " Score"));
        }

        if (manager.hasReachedGoal("L" + level + " Score")) {
            manager.requestNode(GraphParser.getNodeByName("Stow"));
            canEnd = true;
        }

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() { 
        return manager.hasReachedGoal("Stow") && canEnd;
    }

}
