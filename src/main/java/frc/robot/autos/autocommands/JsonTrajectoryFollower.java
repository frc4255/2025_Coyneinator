package frc.robot.autos.autocommands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/**
 * Simple JSON trajectory follower that feeds Pose2d setpoints into the swerve PID controllers.
 */
public class JsonTrajectoryFollower extends Command {
    private final Swerve swerve;
    private final List<Pose2d> path;
    private final double posToleranceMeters;
    private final double headingToleranceRad;
    private final int lookaheadPoints = 2;
    private final int searchWindow = 8;

    private int index = 0;
    private double bestDistance = Double.MAX_VALUE;

    public JsonTrajectoryFollower(Swerve swerve, List<Pose2d> path) {
        this(swerve, path, 0.10, Math.toRadians(5.0));
    }

    public JsonTrajectoryFollower(Swerve swerve, List<Pose2d> path, double posToleranceMeters, double headingToleranceRad) {
        this.swerve = swerve;
        this.path = path;
        this.posToleranceMeters = posToleranceMeters;
        this.headingToleranceRad = headingToleranceRad;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        index = 0;
        bestDistance = Double.MAX_VALUE;
        swerve.setPoseControllerTolerances(posToleranceMeters, headingToleranceRad);
    }

    @Override
    public void execute() {
        if (path.isEmpty()) {
            return;
        }

        Pose2d pose = swerve.getPose();
        int nearest = index;
        double nearestDist = Double.MAX_VALUE;

        // Find the closest upcoming waypoint to our current pose within a limited window (no backtracking).
        int end = Math.min(path.size(), index + searchWindow);
        for (int i = index; i < end; i++) {
            double dist = pose.getTranslation().getDistance(path.get(i).getTranslation());
            if (dist < nearestDist) {
                nearestDist = dist;
                nearest = i;
            }
        }

        // Prevent backwards progress.
        index = Math.max(index, nearest);
        bestDistance = Math.min(bestDistance, nearestDist);

        int targetIdx = Math.min(index + lookaheadPoints, path.size() - 1);
        Pose2d target = path.get(targetIdx);
        swerve.followPose(target);

        double posError = pose.getTranslation().getDistance(target.getTranslation());
        double headingError = Math.abs(pose.getRotation().minus(target.getRotation()).getRadians());
        Logger.recordOutput("JsonFollower/index", index);
        Logger.recordOutput("JsonFollower/targetIdx", targetIdx);
        Logger.recordOutput("JsonFollower/posErrorMeters", posError);
        Logger.recordOutput("JsonFollower/headingErrorRad", headingError);
        Logger.recordOutput("JsonFollower/targetPose", target);
        Logger.recordOutput("JsonFollower/currentPose", pose);
    }

    @Override
    public boolean isFinished() {
        if (path.isEmpty()) {
            return true;
        }
        Pose2d pose = swerve.getPose();
        Pose2d goal = path.get(path.size() - 1);
        double posError = pose.getTranslation().getDistance(goal.getTranslation());
        double headingError = Math.abs(pose.getRotation().minus(goal.getRotation()).getRadians());
        return posError <= posToleranceMeters && headingError <= headingToleranceRad;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0.0, true, false);
    }
}
