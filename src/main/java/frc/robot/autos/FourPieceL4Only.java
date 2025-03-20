package frc.robot.autos;

import frc.robot.subsystems.AlignTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;
import frc.robot.subsystems.EndEffector;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.OnTheFlyTrajectory;
import frc.robot.RobotContainer;
import frc.robot.SubsystemManager;
import frc.robot.commands.CoralHumanPlayerIntake;
import frc.robot.commands.L4Assist;

public class FourPieceL4Only extends SequentialCommandGroup{
    public FourPieceL4Only(Swerve s_Swerve, Pivot s_Pivot, AlignTool AlignTool,
        OnTheFlyTrajectory onTheFlyTrajectory, Elevator s_Elevator, WristPitch s_WristPitch, 
        WristRoll s_WristRoll, EndEffector s_EndEffector, SubsystemManager manager) 
        throws FileVersionException, IOException, ParseException {
        
        PathPlannerPath path0 = PathPlannerPath.fromPathFile("4pc 0");
        PathPlannerPath path1 = PathPlannerPath.fromPathFile("4pc 1");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("4pc 2");
        PathPlannerPath path3 = PathPlannerPath.fromPathFile("4pc 3");
        PathPlannerPath path4 = PathPlannerPath.fromPathFile("4pc 4");
        PathPlannerPath path5 = PathPlannerPath.fromPathFile("4pc 5");
        PathPlannerPath path6 = PathPlannerPath.fromPathFile("4pc 6");

        addCommands(
            new InstantCommand(() -> {
            path0.getStartingHolonomicPose().ifPresentOrElse(
                startingPose -> {
                    Pose2d adjustedPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        ? new Pose2d(
                            startingPose.getTranslation(),
                            startingPose.getRotation().plus(Rotation2d.fromDegrees(180))
                        )
                        : startingPose;
                    s_Swerve.setPose(adjustedPose);
                },
                () -> {
                    System.out.println("Warning: 4 pc auto has no path, Rad and Nick are cooked");
                    }
                );
            }), 

            new WaitCommand(0.1),

            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new L4Assist(manager, s_Swerve, AlignTool, onTheFlyTrajectory).withTimeout(1) //TODO TUNE THIS
                )
            ),

            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path1),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new CoralHumanPlayerIntake(manager, s_EndEffector).withTimeout(4) //TODO TUNE THIS
                )
            ),

            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path2),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new L4Assist(manager, s_Swerve, AlignTool, onTheFlyTrajectory).withTimeout(1) //TODO TUNE THIS
                )
            ),

            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path3),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new CoralHumanPlayerIntake(manager, s_EndEffector).withTimeout(4) //TODO TUNE THIS
                )
            ),

            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path4),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new L4Assist(manager, s_Swerve, AlignTool, onTheFlyTrajectory).withTimeout(1) //TODO TUNE THIS
                )
            ),

            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path5),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new CoralHumanPlayerIntake(manager, s_EndEffector).withTimeout(4) //TODO TUNE THIS
                )
            ),

            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path6),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new L4Assist(manager, s_Swerve, AlignTool, onTheFlyTrajectory).withTimeout(1) //TODO TUNE THIS
                )
            )


        );




    }
}
