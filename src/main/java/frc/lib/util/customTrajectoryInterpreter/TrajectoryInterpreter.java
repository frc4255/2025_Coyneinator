package frc.lib.util.customTrajectoryInterpreter;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

public class TrajectoryInterpreter {

    public static class Point {
        public double x;
        public double y;
        public double heading;

        // Jackson needs a no-arg constructor
        public Point() {}

        public Point(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        @Override
        public String toString() {
            return String.format("Point{x=%.3f, y=%.3f, heading=%.3f}", x, y, heading);
        }
    }

    public static void main(String[] args) throws IOException {
        File file = new File("src\\main\\deploy\\customAutos\\CirclePath.json");

        // Jackson object mapper
        ObjectMapper mapper = new ObjectMapper();

        // Read JSON into Map<String, List<Point>>
        TypeReference<Map<String, List<Point>>> typeRef =
                new TypeReference<Map<String, List<Point>>>() {};

        Map<String, List<Point>> trajectoriesByName = mapper.readValue(file, typeRef);

        // ---- Example 1: Access a specific trajectory by name ----
        List<Point> traj1 = trajectoriesByName.get("traj_1");
        if (traj1 != null) {
            System.out.println("traj_1 has " + traj1.size() + " points");
            Point first = traj1.get(0);
            System.out.println("First point of traj_1:");
            System.out.println("x = " + first.x);
            System.out.println("y = " + first.y);
            System.out.println("heading = " + first.heading);
        } else {
            System.out.println("traj_1 not found in JSON");
        }

        // ---- Example 2: Sort traj_1, traj_2, ..., traj_n numerically ----
        List<Map.Entry<String, List<Point>>> sortedEntries =
                trajectoriesByName.entrySet()
                        .stream()
                        .sorted(Comparator.comparing(e ->
                                parseTrajIndex(e.getKey()))) // "traj_5" -> 5
                        .toList();

        // Build a list [traj_1, traj_2, ..., traj_n] as List<List<Point>>
        List<List<Point>> trajectoriesInOrder = new ArrayList<>();
        for (Map.Entry<String, List<Point>> entry : sortedEntries) {
            String name = entry.getKey();
            List<Point> points = entry.getValue();
            trajectoriesInOrder.add(points);

            System.out.println("\nTrajectory: " + name + " (" + points.size() + " points)");
            // Show a few points
            for (int i = 0; i < Math.min(3, points.size()); i++) {
                Point p = points.get(i);
                System.out.println("  i=" + i + " -> x=" + p.x + ", y=" + p.y + ", heading=" + p.heading);
            }
        }

        // ---- Example 3: Use trajectoriesInOrder like [traj_1, traj_2, ..., traj_n] ----
        if (!trajectoriesInOrder.isEmpty()) {
            List<Point> firstTrajectory = trajectoriesInOrder.get(0); // traj_1 if naming is consistent
            System.out.println("\nFirst trajectory in ordered list has " + firstTrajectory.size() + " points.");
        }
    }

    // Helper: convert "traj_7" -> 7, default to Integer.MAX_VALUE if format is weird
    private static int parseTrajIndex(String key) {
        try {
            return Integer.parseInt(key.replace("traj_", "").trim());
        } catch (NumberFormatException e) {
            return Integer.MAX_VALUE;
        }
    }
}
