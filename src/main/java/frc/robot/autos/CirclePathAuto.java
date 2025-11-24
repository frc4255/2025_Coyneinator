package frc.robot.autos;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.customTrajectoryInterpreter.JsonTrajectorySet;
import frc.robot.autos.autocommands.JsonTrajectoryFollower;
import frc.robot.subsystems.Swerve;

public class CirclePathAuto extends SequentialCommandGroup {
    private static final String FILE_NAME = "CirclePath.json";

    public CirclePathAuto(Swerve swerve) {
        boolean flipForRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        JsonTrajectorySet trajectories;

        try {
            trajectories = JsonTrajectorySet.fromDeployFile(FILE_NAME);
        } catch (IOException e) {
            addCommands(new InstantCommand(() ->
                DriverStation.reportError("Failed to load " + FILE_NAME + ": " + e.getMessage(), e.getStackTrace()))
            );
            return;
        }

        Optional<Pose2d> initialPose = trajectories.getInitialPose(flipForRed);
        initialPose.ifPresent(pose -> addCommands(
            new InstantCommand(() -> {
                swerve.setHeading(pose.getRotation());
                swerve.setPose(pose);
            }),
            new WaitCommand(0.1)
        ));

        List<List<Pose2d>> paths = trajectories.getTrajectoriesInOrder(flipForRed);
        for (int i = 0; i < paths.size(); i++) {
            List<Pose2d> path = paths.get(i);
            addCommands(new JsonTrajectoryFollower(swerve, path));

            // Break between segments so game-piece actions can be inserted.
            if (i < paths.size() - 1) {
                addCommands(new WaitCommand(0.25));
            }
        }
    }
}
