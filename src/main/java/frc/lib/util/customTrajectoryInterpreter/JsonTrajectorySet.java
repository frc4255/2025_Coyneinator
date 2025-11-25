package frc.lib.util.customTrajectoryInterpreter;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.util.FlippingUtil;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Loads JSON trajectories (e.g., CirclePath.json) and exposes them as Pose2d lists.
 * Supports alliance flipping and ordered access to traj_1, traj_2, ... segments.
 */
public class JsonTrajectorySet {

    public record TimedTrajectory(String name, List<Pose2d> poses, double totalTimeSeconds) {}

    private record Point(double x, double y, double heading) {}
    private record TimingSection(int section, double time) {}
    private record TimingInfo(double total, List<TimingSection> sections) {}
    private record TrajectoryDocument(TimingInfo times, Map<String, List<Point>> trajectories) {}

    private final Map<String, List<Point>> trajectoriesByName;
    private final Map<String, Double> trajectoryDurations;
    private final List<String> orderedNames;

    private JsonTrajectorySet(Map<String, List<Point>> trajectoriesByName, Map<String, Double> trajectoryDurations) {
        this.trajectoriesByName = trajectoriesByName;
        this.trajectoryDurations = trajectoryDurations;
        this.orderedNames = trajectoriesByName.keySet()
            .stream()
            .sorted(Comparator.comparingInt(JsonTrajectorySet::parseTrajIndex))
            .toList();
    }

    public static JsonTrajectorySet fromDeployFile(String filename) throws IOException {
        File fileOnRobot = new File(Filesystem.getDeployDirectory(), "customAutos/" + filename);
        ObjectMapper mapper = new ObjectMapper();
        TrajectoryDocument document = mapper.readValue(fileOnRobot, TrajectoryDocument.class);
        Map<String, Double> durations = buildDurationMap(document.times());
        return new JsonTrajectorySet(document.trajectories(), durations);
    }

    /**
     * First non-empty trajectory's first point, flipped for alliance if requested.
     */
    public Optional<Pose2d> getInitialPose(boolean flipForRed) {
        for (String name : orderedNames) {
            List<Point> pts = trajectoriesByName.get(name);
            if (pts != null && !pts.isEmpty()) {
                return Optional.of(toPose(pts.get(0), flipForRed));
            }
        }
        return Optional.empty();
    }

    /**
     * Returns the specified trajectory as Pose2d list (empty if missing).
     */
    public List<Pose2d> getTrajectory(String name, boolean flipForRed) {
        List<Point> pts = trajectoriesByName.get(name);
        if (pts == null) {
            return List.of();
        }
        List<Pose2d> out = new ArrayList<>(pts.size());
        for (Point pt : pts) {
            out.add(toPose(pt, flipForRed));
        }
        return out;
    }

    /**
     * Trajectories returned in traj_1, traj_2, ... order with alliance flip applied.
     */
    public List<List<Pose2d>> getTrajectoriesInOrder(boolean flipForRed) {
        return getTimedTrajectoriesInOrder(flipForRed).stream()
            .map(TimedTrajectory::poses)
            .toList();
    }

    public List<TimedTrajectory> getTimedTrajectoriesInOrder(boolean flipForRed) {
        List<TimedTrajectory> out = new ArrayList<>();
        for (String name : orderedNames) {
            List<Pose2d> traj = getTrajectory(name, flipForRed);
            if (!traj.isEmpty()) {
                double duration = trajectoryDurations.getOrDefault(name, 0.0);
                out.add(new TimedTrajectory(name, traj, duration));
            }
        }
        return out;
    }

    public List<String> getOrderedNames() {
        return orderedNames;
    }

    private Pose2d toPose(Point point, boolean flipForRed) {
        Pose2d pose = new Pose2d(point.x, point.y, new Rotation2d(point.heading));
        return flipForRed ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    private static int parseTrajIndex(String key) {
        try {
            return Integer.parseInt(key.replace("traj_", "").trim());
        } catch (NumberFormatException e) {
            return Integer.MAX_VALUE;
        }
    }

    private static Map<String, Double> buildDurationMap(TimingInfo timingInfo) {
        Map<String, Double> result = new HashMap<>();
        if (timingInfo == null || timingInfo.sections() == null) {
            return result;
        }
        for (TimingSection section : timingInfo.sections()) {
            String trajName = "traj_" + section.section();
            result.put(trajName, section.time());
        }
        return result;
    }
}
