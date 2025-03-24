package frc.robot.autos;

import frc.robot.subsystems.AlignTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;
import frc.robot.subsystems.EndEffector;

import java.io.IOException;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.SubsystemManager;
import frc.robot.commands.CoralHumanPlayerIntake;
import frc.robot.commands.ExtakeCoral;
import frc.robot.commands.L1Assist;
import frc.robot.commands.L4Assist;
import frc.robot.commands.Stow;

public class OnePieceL1 extends SequentialCommandGroup{
    public OnePieceL1(Swerve s_Swerve, AlignTool alignTool,
        Pivot s_Pivot, Elevator s_Elevator, WristPitch s_WristPitch, 
        WristRoll s_WristRoll, EndEffector s_EndEffector, SubsystemManager manager, AutoFactory factory) {
        
        Command myTrajectory = factory.trajectoryCmd("test");

        addCommands(
            factory.resetOdometry("test"),
            new WaitCommand(0.1),
            factory.trajectoryCmd("test"),
            new ParallelCommandGroup(
                //s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new L1Assist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll).withTimeout(3),
                    new ExtakeCoral(s_EndEffector).withTimeout(0.5), //TODO TUNE THIS
                    new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
                    
                )
            )


        );




    }
}
