package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
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
    private EndEffector s_EndEffector;

    //Relative to apriltag
    private Pose2d leftScoringPose; //odd
    private Pose2d rightScoringPose; //even
    private int level;

    private boolean canEnd;

    private final PIDController xController;
    private final  PIDController yController;
    private final PIDController headingController;

    private Pose2d fieldTargetPose;
    private Pose2d activeTargetPose;

    private BooleanSupplier confirmSupplier;

    public Score(int level, SubsystemManager manager, Pivot s_Pivot, 
        Elevator s_Elevator, WristPitch s_WristPitch, WristRoll s_WristRoll, Swerve s_Swerve, EndEffector s_EndEffector, BooleanSupplier confirmSupplier) {

        this.manager = manager;

        this.s_Pivot = s_Pivot;
        this.s_Elevator = s_Elevator;
        this.s_WristPitch = s_WristPitch;
        this.s_WristRoll = s_WristRoll;
        this.s_Swerve = s_Swerve;
        this.s_EndEffector = s_EndEffector;

        this.level = level;

        this.confirmSupplier = confirmSupplier;

        leftScoringPose = new Pose2d(new Translation2d(0.447,-0.16), new Rotation2d());
        rightScoringPose = new Pose2d(new Translation2d(0.447, 0.16), new Rotation2d());

        xController = new PIDController(2.0, 0.0, 0.0);
        yController = new PIDController(2.0, 0.0, 0.0);
        headingController = new PIDController(1, 0.0, 0.0);

        xController.setTolerance(0.08);
        yController.setTolerance(0.08);
        headingController.setTolerance(0.1);
        headingController.enableContinuousInput(-Math.PI, Math.PI);


        addRequirements(s_Pivot, s_Elevator, s_WristPitch, s_WristRoll, s_Swerve);
    }

    @Override
    public void initialize() {
        manager.requestNode("Reef Align");

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        sector = s_Swerve.findClosestBranch(alliance == Alliance.Red);
        int branchNumber = frc.robot.FieldLayout.Reef.branchesToInt.get(sector);
        activeTargetPose = (branchNumber % 2 == 1) ? leftScoringPose : rightScoringPose;

        int tagID = FieldLayout.Reef.branchesToAprilTagID.get(sector);

        Logger.recordOutput("Target Tag ID", tagID);
        Pose3d tagPose3d = FieldLayout.AprilTags.APRIL_TAG_POSE.stream()
            .filter(tag -> tag.ID == tagID)
            .findFirst()
            .orElseThrow()
            .pose;
        Pose2d tagPose2d = new Pose2d(tagPose3d.getX(), tagPose3d.getY(), new Rotation2d(tagPose3d.getRotation().getZ()));
        fieldTargetPose = tagPose2d.transformBy(new Transform2d(new Translation2d(activeTargetPose.getX(), activeTargetPose.getY()), activeTargetPose.getRotation()));

        xController.setSetpoint(fieldTargetPose.getTranslation().getX());
        yController.setSetpoint(fieldTargetPose.getTranslation().getY());
        headingController.setSetpoint(fieldTargetPose.getTranslation().getAngle().getRadians());
    }

    @Override
    public void execute() {

        // Get the current robot pose from s_Swerve
        Pose2d currentPose = s_Swerve.getPose();
        
        // Transform activeTargetPose from apriltag-relative coordinates to field coordinates
        

        // Calculate PID outputs for x, y, and heading to move towards the field target pose
        double xCommand = xController.calculate(currentPose.getTranslation().getX());
        double yCommand = yController.calculate(currentPose.getTranslation().getY());
        double headingCommand = headingController.calculate(currentPose.getRotation().getRadians());
        
        // Command the swerve drive to move towards the target
        s_Swerve.drive(new Translation2d(xCommand, yCommand), headingCommand, true, false);

        if (xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint()) {
            manager.requestNode("L" + level + " Init");
            s_Swerve.drive(new Translation2d(0,0), 0, false, false);
        }

        if (manager.hasReachedGoal("L" + String.valueOf(level) + " Init") && confirmSupplier.getAsBoolean()) {
            manager.requestNode("Stow");
            s_EndEffector.setDutyCycle(2);
            canEnd = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        s_EndEffector.setDutyCycle(0);

    }

    @Override
    public boolean isFinished() { 
        return manager.hasReachedGoal("Reef Align") && canEnd;
    }

}
