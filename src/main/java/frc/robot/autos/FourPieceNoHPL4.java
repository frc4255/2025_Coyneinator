package frc.robot.autos;

import frc.robot.subsystems.AlignTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;
import frc.robot.subsystems.EndEffector;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SubsystemManager;
import frc.robot.autos.autocommands.IntakeThenReefAlignWhenCurrentAction;
import frc.robot.autos.autocommands.LollipopIntake;
import frc.robot.commands.CoralHumanPlayerIntake;
import frc.robot.commands.L4Assist;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.Stow;

public class FourPieceNoHPL4 extends SequentialCommandGroup{
    public FourPieceNoHPL4(Swerve s_Swerve, Pivot s_Pivot, AlignTool AlignTool, Elevator s_Elevator, WristPitch s_WristPitch, 
        WristRoll s_WristRoll, EndEffector s_EndEffector, SubsystemManager manager) {

        addCommands(
            //new InstantCommand(() -> s_Swerve.setPose(path0.getStartingHolonomicPose().get())), 
            new WaitCommand(0.1),
            new ParallelCommandGroup(
              //  s_Swerve.getPathCommand(path0)
                //new ReefAlign(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
            ),
            new L4Assist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll),
            new ParallelCommandGroup(
                new IntakeThenReefAlignWhenCurrentAction(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll),
                new SequentialCommandGroup(
                    new WaitCommand(0.8)
                   // s_Swerve.getPathCommand(path1)
                )
            ),
            new L4Assist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll),
            new ParallelCommandGroup(
                new IntakeThenReefAlignWhenCurrentAction(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll),
                new SequentialCommandGroup(
                    new WaitCommand(0.8) //TODO TUNE THIS
                   // s_Swerve.getPathCommand(path2)
                )
            ),
            new L4Assist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll),
            new ParallelCommandGroup(
                new IntakeThenReefAlignWhenCurrentAction(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll),
                new SequentialCommandGroup(
                    new WaitCommand(0.8)//TODO TUNE THIS
                   // s_Swerve.getPathCommand(path3)
                )
            ),
            new L4Assist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll),
            new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
            
        );
    }
}
