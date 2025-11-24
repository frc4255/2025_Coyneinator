package frc.lib.util.customTrajectoryInterpreter;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.util.FlippingUtil;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * Loads JSON trajectories (e.g., CirclePath.json) and exposes them as Pose2d lists.
 * Supports alliance flipping and ordered access to traj_1, traj_2, ... segments.
 */
public class JsonTrajectorySet {

    private record Point(double x, double y, double heading) {}

    private final Map<String, List<Point>> trajectoriesByName;
    private final List<String> orderedNames;

    private JsonTrajectorySet(Map<String, List<Point>> trajectoriesByName) {
        this.trajectoriesByName = trajectoriesByName;
        this.orderedNames = trajectoriesByName.keySet()
            .stream()
            .sorted(Comparator.comparingInt(JsonTrajectorySet::parseTrajIndex))
            .toList();
    }

    public static JsonTrajectorySet fromDeployFile(String filename) throws IOException {
        File fileOnRobot = new File(Filesystem.getDeployDirectory(), "customAutos/" + filename);
        ObjectMapper mapper = new ObjectMapper();
        TypeReference<Map<String, List<Point>>> typeRef = new TypeReference<>() {};
        Map<String, List<Point>> raw = mapper.readValue(fileOnRobot, typeRef);
        return new JsonTrajectorySet(raw);
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
        List<List<Pose2d>> out = new ArrayList<>();
        for (String name : orderedNames) {
            List<Pose2d> traj = getTrajectory(name, flipForRed);
            if (!traj.isEmpty()) {
                out.add(traj);
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
}
