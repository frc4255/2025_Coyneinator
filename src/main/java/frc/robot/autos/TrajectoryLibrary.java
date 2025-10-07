package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Central location for creating and caching autonomous trajectories and PathPlanner paths.
 * This makes it significantly easier to tweak or add routines without touching command code.
 */
public class TrajectoryLibrary {
    private final Supplier<TrajectoryConfig> configSupplier;
    private final SwerveDriveKinematics kinematics;
    private final double kPX;
    private final double kPY;
    private final double kPTheta;
    private final TrapezoidProfile.Constraints thetaConstraints;

    private final Map<String, Trajectory> trajectoryCache = new HashMap<>();

    public TrajectoryLibrary(
            Supplier<TrajectoryConfig> configSupplier,
            SwerveDriveKinematics kinematics,
            double kPX,
            double kPY,
            double kPTheta,
            TrapezoidProfile.Constraints thetaConstraints) {
        this.configSupplier = configSupplier;
        this.kinematics = kinematics;
        this.kPX = kPX;
        this.kPY = kPY;
        this.kPTheta = kPTheta;
        this.thetaConstraints = thetaConstraints;
    }

    public TrajectoryConfig newConfig() {
        return configSupplier.get();
    }

    public Optional<Trajectory> getTrajectory(String id) {
        return Optional.ofNullable(trajectoryCache.get(id));
    }

    public Trajectory generateTrajectory(
            String id,
            Pose2d start,
            List<Translation2d> interior,
            Pose2d end) {
        return trajectoryCache.computeIfAbsent(
            id,
            key -> TrajectoryGenerator.generateTrajectory(start, interior, end, newConfig())
        );
    }

    public Trajectory regenerateTrajectory(
            String id,
            Pose2d start,
            List<Translation2d> interior,
            Pose2d end) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, interior, end, newConfig());
        trajectoryCache.put(id, trajectory);
        return trajectory;
    }

    public Trajectory generateTrajectory(
            String id,
            Pose2d start,
            List<Translation2d> interior,
            Pose2d end,
            TrajectoryConfig config) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, interior, end, config);
        trajectoryCache.put(id, trajectory);
        return trajectory;
    }

    public void clearTrajectory(String id) {
        trajectoryCache.remove(id);
    }

    public void clearAllTrajectories() {
        trajectoryCache.clear();
    }

    public Command buildFollowPathCommand(String pathName, Swerve swerve) {
        return Commands.print("PathPlanner path unavailable: " + pathName);
    }

    public Command buildTrajectoryCommand(
            String id,
            Swerve swerve,
            Pose2d start,
            List<Translation2d> interior,
            Pose2d end) {
        Trajectory trajectory = generateTrajectory(id, start, interior, end);
        return buildTrajectoryCommand(id, swerve, trajectory);
    }

    public Command buildTrajectoryCommand(String id, Swerve swerve, Trajectory trajectory) {
        trajectoryCache.put(id, trajectory);
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerve.setPose(trajectory.getInitialPose()), swerve),
            createHolonomicCommand(swerve, trajectory)
        );
    }

    public Command createHolonomicCommand(Swerve swerve, Trajectory trajectory) {
        PIDController xController = new PIDController(kPX, 0, 0);
        PIDController yController = new PIDController(kPY, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(kPTheta, 0, 0, thetaConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            kinematics,
            xController,
            yController,
            thetaController,
            swerve::setModuleStates,
            swerve
        );
    }
}
