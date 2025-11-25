package frc.robot.autos.autocommands;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/**
 * JSON trajectory follower that advances along the path based on the total time supplied in the JSON.
 * <p>Every 20 ms the command advances the expected distance along the polyline so the drivetrain knows
 * which pose it should be at for that exact instant. The path itself is never re-ordered; we simply
 * interpolate between successive samples to keep the target moving smoothly.</p>
 */
public class JsonTrajectoryFollower extends Command {
    private static final double MIN_TOTAL_TIME = 1e-3;

    private record PathSample(Pose2d pose, double distanceMeters) {}

    private final Swerve swerve;
    private final List<PathSample> samples;
    private final double pathLengthMeters;
    private final double totalTimeSeconds;
    private final double posToleranceMeters;
    private final double headingToleranceRad;
    private final Pose2d goalPose;
    private final Timer timer = new Timer();

    public JsonTrajectoryFollower(Swerve swerve, List<Pose2d> path, double totalTimeSeconds) {
        this(swerve, path, totalTimeSeconds, 0.10, Math.toRadians(5.0));
    }

    public JsonTrajectoryFollower(
            Swerve swerve,
            List<Pose2d> path,
            double totalTimeSeconds,
            double posToleranceMeters,
            double headingToleranceRad
    ) {
        this.swerve = swerve;
        this.samples = buildSamples(path);
        this.pathLengthMeters = samples.isEmpty() ? 0.0 : samples.get(samples.size() - 1).distanceMeters();
        this.totalTimeSeconds = Math.max(totalTimeSeconds, MIN_TOTAL_TIME);
        this.posToleranceMeters = posToleranceMeters;
        this.headingToleranceRad = headingToleranceRad;
        this.goalPose = samples.isEmpty() ? new Pose2d() : samples.get(samples.size() - 1).pose();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        swerve.setPoseControllerTolerances(posToleranceMeters, headingToleranceRad);
    }

    @Override
    public void execute() {
        if (samples.isEmpty()) {
            return;
        }

        double elapsed = Math.min(timer.get(), totalTimeSeconds);
        double progress = elapsed / totalTimeSeconds;
        double targetDistance = progress * pathLengthMeters;

        Pose2d targetPose = samplePoseAt(targetDistance);
        Pose2d currentPose = swerve.getPose();
        swerve.followPose(targetPose);

        double posError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double headingError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        Logger.recordOutput("JsonFollower/elapsedTimeSeconds", elapsed);
        Logger.recordOutput("JsonFollower/totalTimeSeconds", totalTimeSeconds);
        Logger.recordOutput("JsonFollower/targetDistanceMeters", targetDistance);
        Logger.recordOutput("JsonFollower/posErrorMeters", posError);
        Logger.recordOutput("JsonFollower/headingErrorRad", headingError);
        Logger.recordOutput("JsonFollower/targetPose", targetPose);
        Logger.recordOutput("JsonFollower/currentPose", currentPose);
    }

    @Override
    public boolean isFinished() {
        if (samples.isEmpty()) {
            return true;
        }
        if (!timer.hasElapsed(totalTimeSeconds)) {
            return false;
        }
        Pose2d pose = swerve.getPose();
        double posError = pose.getTranslation().getDistance(goalPose.getTranslation());
        double headingError = Math.abs(pose.getRotation().minus(goalPose.getRotation()).getRadians());
        return posError <= posToleranceMeters && headingError <= headingToleranceRad;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.drive(new Translation2d(), 0.0, true, false);
    }

    private static List<PathSample> buildSamples(List<Pose2d> path) {
        List<PathSample> built = new ArrayList<>();
        if (path.isEmpty()) {
            return built;
        }

        double cumulative = 0.0;
        built.add(new PathSample(path.get(0), 0.0));
        for (int i = 1; i < path.size(); i++) {
            Pose2d prev = path.get(i - 1);
            Pose2d current = path.get(i);
            double segment = prev.getTranslation().getDistance(current.getTranslation());
            cumulative += segment;
            built.add(new PathSample(current, cumulative));
        }
        return built;
    }

    private Pose2d samplePoseAt(double distanceMeters) {
        if (samples.isEmpty()) {
            return new Pose2d();
        }
        if (distanceMeters <= 0.0 || samples.size() == 1 || pathLengthMeters <= 1e-6) {
            return samples.get(0).pose();
        }
        if (distanceMeters >= pathLengthMeters) {
            return samples.get(samples.size() - 1).pose();
        }

        for (int i = 0; i < samples.size() - 1; i++) {
            PathSample start = samples.get(i);
            PathSample end = samples.get(i + 1);
            if (distanceMeters <= end.distanceMeters()) {
                double span = end.distanceMeters() - start.distanceMeters();
                if (span < 1e-6) {
                    return end.pose();
                }
                double t = (distanceMeters - start.distanceMeters()) / span;
                return start.pose().interpolate(end.pose(), t);
            }
        }
        return samples.get(samples.size() - 1).pose();
    }
}
