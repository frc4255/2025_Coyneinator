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
import frc.robot.commands.L1Assist;
import frc.robot.commands.L4Assist;

public class OnePieceL1 extends SequentialCommandGroup{
    public OnePieceL1(Swerve s_Swerve, AlignTool alignTool, OnTheFlyTrajectory onTheFlyTrajectory, 
        Pivot s_Pivot, Elevator s_Elevator, WristPitch s_WristPitch, 
        WristRoll s_WristRoll, EndEffector s_EndEffector, SubsystemManager manager) 
        throws FileVersionException, IOException, ParseException {
        
        PathPlannerPath path0 = PathPlannerPath.fromPathFile("1Pc");

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
                    System.out.println("Warning: 1 pc auto has no path, Rad and Nick are cooked");
                    }
                );
            }), 

            new WaitCommand(0.1),

            new ParallelCommandGroup(
                s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new L1Assist(manager, s_Swerve, alignTool, onTheFlyTrajectory).withTimeout(1) //TODO TUNE THIS
                )
            )


        );




    }
}
