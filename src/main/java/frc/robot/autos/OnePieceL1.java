package frc.robot.autos;

import frc.robot.subsystems.AlignTool;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.DifferentialWrist;
import frc.robot.subsystems.EndEffector;

import java.io.IOException;
import java.util.Optional;

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
import frc.robot.commands.Stow;

public class OnePieceL1 extends SequentialCommandGroup{
    public OnePieceL1(
            Swerve s_Swerve,
            AlignTool alignTool,
            Pivot s_Pivot,
            Elevator s_Elevator,
            DifferentialWrist s_Wrist,
            EndEffector s_EndEffector,
            SubsystemManager manager
    ) {
        
        Optional<Trajectory<SwerveSample>> optionalTrajectory = Choreo.loadTrajectory("test");
        Trajectory<SwerveSample> myTrajectory = optionalTrajectory.get();


        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(optionalTrajectory.get().getInitialPose(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)).get())),
            new WaitCommand(0.1),
            new SwerveFollower(s_Swerve, myTrajectory),
            new ParallelCommandGroup(
                //s_Swerve.followPathCommand(path0),
                new SequentialCommandGroup(
                    new WaitCommand(2.4), //TODO TUNE THIS
                    new L1Assist(manager, s_Pivot, s_Elevator, s_Wrist).withTimeout(3),
                    new ExtakeCoral(s_EndEffector).withTimeout(0.5), //TODO TUNE THIS
                    new Stow(manager, s_Pivot, s_Elevator, s_Wrist)
                )
            )


        );




    }
}
