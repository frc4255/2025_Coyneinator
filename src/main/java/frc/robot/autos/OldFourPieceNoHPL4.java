package frc.robot.autos;

import frc.robot.subsystems.AlignTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;
import frc.robot.subsystems.EndEffector;

import java.io.IOException;
import java.util.HashMap;
import java.util.Optional;
import java.util.ArrayList;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.SubsystemManager;
import frc.robot.autos.autocommands.SwerveFollower;
import frc.robot.commands.CoralHumanPlayerIntake;
import frc.robot.commands.ExtakeCoral;
import frc.robot.commands.L1Assist;
import frc.robot.commands.L4Assist;
import frc.robot.commands.LollipopIntakeManual;
import frc.robot.commands.Score;
import frc.robot.commands.Stow;

public class OldFourPieceNoHPL4 extends SequentialCommandGroup{
    public OldFourPieceNoHPL4(Swerve s_Swerve, AlignTool alignTool,
        Pivot s_Pivot, Elevator s_Elevator, WristPitch s_WristPitch, 
        WristRoll s_WristRoll, EndEffector s_EndEffector, SubsystemManager manager) {
        
        ArrayList<Optional<Trajectory<SwerveSample>>> trajectories = new ArrayList<>();

        trajectories.add(Choreo.loadTrajectory("4pc_0 (Start-I)"));
        trajectories.add(Choreo.loadTrajectory("4pc_1 (I-A)"));
        trajectories.add(Choreo.loadTrajectory("4pc_2 (A-B)"));
        trajectories.add(Choreo.loadTrajectory("4pc_3 (B-C)"));


        Optional<Pose2d> initialPose = trajectories.get(0).get().getInitialPose(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red));

        //s_Swerve.followTrajectory(myTrajectory.sampleAt(
        //new Timer().get(), DriverStation.getAlliance().orElse(Alliance.Blue)
        //.equals(Alliance.Red)).get())),

        addCommands(
            new InstantCommand(() -> s_Swerve.setHeading(initialPose.get().getRotation())),
            new InstantCommand(() -> s_Swerve.setPose(initialPose.get())),
            new WaitCommand(0.1),
            
            new SwerveFollower(s_Swerve, trajectories.get(0).get()),

            
            new ParallelCommandGroup(
                //s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new Score(4, manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll, s_Swerve).withTimeout(3),
                    new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)                    
                )
            ),
             

            new ParallelCommandGroup(
                new SwerveFollower(s_Swerve, trajectories.get(1).get()),
                
                new WaitCommand(1),
                new SequentialCommandGroup(
                    new LollipopIntakeManual(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll).withTimeout(2),
                    new ExtakeCoral(s_EndEffector).withTimeout(0.5),
                    new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
                )
            ),
            
            new SequentialCommandGroup(
                new Score(4, manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll, s_Swerve).withTimeout(2),
                new ExtakeCoral(s_EndEffector).withTimeout(0.5),
                new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
            ),  


            new ParallelCommandGroup(
                new SwerveFollower(s_Swerve, trajectories.get(2).get()),
                new WaitCommand(0.75),
                new SequentialCommandGroup(
                    new LollipopIntakeManual(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll).withTimeout(2),
                    new ExtakeCoral(s_EndEffector).withTimeout(0.5),
                    new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
                ) 
            ),
            
            new SequentialCommandGroup(
                new Score(4, manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll, s_Swerve).withTimeout(2),
                new ExtakeCoral(s_EndEffector).withTimeout(0.5),
                new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
            ), 
            
    
            new ParallelCommandGroup(
                new SwerveFollower(s_Swerve, trajectories.get(3).get()),

                
                new WaitCommand(1.1),
                new SequentialCommandGroup(
                    new LollipopIntakeManual(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll).withTimeout(2),
                    new ExtakeCoral(s_EndEffector).withTimeout(0.5),
                    new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
                ) 
            ),
            
            new SequentialCommandGroup(
                new Score(4, manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll, s_Swerve).withTimeout(2),  
                new ExtakeCoral(s_EndEffector).withTimeout(0.5),     
                new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
            ) 
            
        );


    }
}
