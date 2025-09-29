package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class exampleAuto extends SequentialCommandGroup {
    private static final String TRAJECTORY_ID = "ExampleAuto";

    public exampleAuto(Swerve swerve, TrajectoryLibrary trajectories){
        TrajectoryConfig config = trajectories.newConfig();
        Trajectory trajectory = trajectories.generateTrajectory(
            TRAJECTORY_ID,
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config
        );

        addCommands(trajectories.buildTrajectoryCommand(TRAJECTORY_ID, swerve, trajectory));
    }
}
