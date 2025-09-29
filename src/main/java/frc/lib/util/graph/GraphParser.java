package frc.lib.util.graph;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.stream.Collectors;

public class GraphParser {

    private static final List<String> DEFAULT_SUBSYSTEM_ORDER =
        List.of("pivot", "elevator", "wristPitch", "wristRoll");
    private static final String DEFAULT_RESOURCE_PATH = "/frc/lib/util/graph/graph_data.json";
    private static final Path FALLBACK_FILESYSTEM_PATH =
        Path.of("src", "main", "resources", "frc", "lib", "util", "graph", "graph_data.json");

    // A lookup table for Node objects (for O(1) access)
    private static final Map<String, Node> nodeLookup = new HashMap<>();

    // This is the class we use in our code.
    public static class GraphData {
        public List<String> subsystemKeys = DEFAULT_SUBSYSTEM_ORDER;
        public List<Node> nodes = List.of();
        public List<Edge> edges = List.of();
        public Map<String, Map<String, List<Node>>> shortestPaths = Map.of();
    }

    // This class is used only for JSON deserialization.
    public static class GraphDataRaw {
        public List<String> subsystems;
        public List<NodeDefinition> nodes;
        public List<EdgeDefinition> edges;
        public Map<String, Map<String, List<String>>> shortestPaths;
    }

    public static class NodeDefinition {
        public String name;
        public Map<String, Double> setpoints;
        public List<Double> legacySetpoints;
    }

    public static class EdgeDefinition {
        public String start;
        public String end;
        public String description;
    }

    private static GraphData graphData;

    // Static initializer: load the graph once when the class is first referenced.
    static {
        loadDefaultGraph();
    }

    private static void loadDefaultGraph() {
        if (!loadGraphFromResource(DEFAULT_RESOURCE_PATH)) {
            loadGraph(FALLBACK_FILESYSTEM_PATH.toString());
        }
    }

    private static boolean loadGraphFromResource(String resourcePath) {
        try (InputStream stream = GraphParser.class.getResourceAsStream(resourcePath)) {
            if (stream == null) {
                return false;
            }
            loadGraph(stream);
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Loads the graph from the JSON file. It first deserializes the JSON into a GraphDataRaw
     * (which expects shortestPaths as lists of String) and then converts it into our GraphData
     * with shortestPaths as lists of Node.
     *
     * @param filePath The path to the JSON file.
     */
    private static void loadGraph(String filePath) {
        if (filePath == null || filePath.isBlank()) {
            loadDefaultGraph();
            return;
        }

        Path path = Path.of(filePath);
        if (!Files.exists(path)) {
            System.err.println("GraphParser: Could not find graph definition at " + filePath);
            loadDefaultGraph();
            return;
        }

        try (InputStream stream = Files.newInputStream(path)) {
            loadGraph(stream);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void loadGraph(InputStream stream) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        GraphDataRaw raw = mapper.readValue(stream, GraphDataRaw.class);
        applyRawGraph(raw);
    }

    private static void applyRawGraph(GraphDataRaw raw) {
        graphData = new GraphData();
        nodeLookup.clear();

        List<String> subsystemKeys =
            raw != null && raw.subsystems != null && !raw.subsystems.isEmpty()
                ? List.copyOf(raw.subsystems)
                : DEFAULT_SUBSYSTEM_ORDER;
        graphData.subsystemKeys = subsystemKeys;

        if (raw != null && raw.nodes != null) {
            List<Node> nodes = raw.nodes.stream()
                .map(node -> toNode(node, subsystemKeys))
                .collect(Collectors.toUnmodifiableList());
            graphData.nodes = nodes;
            for (Node node : nodes) {
                nodeLookup.put(node.getName(), node);
            }
        } else {
            graphData.nodes = List.of();
        }

        if (raw != null && raw.edges != null) {
            graphData.edges = raw.edges.stream()
                .map(edge -> new Edge(edge.start, edge.end, edge.description))
                .collect(Collectors.toUnmodifiableList());
        } else {
            graphData.edges = List.of();
        }

        graphData.shortestPaths = convertShortestPaths(
            raw == null ? null : raw.shortestPaths
        );
    }

    private static Node toNode(NodeDefinition rawNode, List<String> subsystemKeys) {
        Map<String, Double> resolved = new LinkedHashMap<>();
        if (rawNode.setpoints != null) {
            resolved.putAll(rawNode.setpoints);
        }
        if (rawNode.legacySetpoints != null && !rawNode.legacySetpoints.isEmpty()) {
            for (int i = 0; i < Math.min(rawNode.legacySetpoints.size(), subsystemKeys.size()); i++) {
                resolved.putIfAbsent(subsystemKeys.get(i), rawNode.legacySetpoints.get(i));
            }
        }
        return new Node(rawNode.name, resolved);
    }

    /**
     * Converts the shortestPaths map from a map of lists of String (node names)
     * to a map of lists of Node objects using our lookup table.
     *
     * @param stringPaths The raw shortestPaths from the JSON.
     * @return The converted map with Node objects.
     */
    private static Map<String, Map<String, List<Node>>> convertShortestPaths(
            Map<String, Map<String, List<String>>> stringPaths) {
        if (stringPaths == null || stringPaths.isEmpty()) {
            return Map.of();
        }

        Map<String, Map<String, List<Node>>> convertedPaths = new HashMap<>();

        for (Map.Entry<String, Map<String, List<String>>> entry : stringPaths.entrySet()) {
            String startNode = entry.getKey();
            Map<String, List<String>> destPaths = entry.getValue();

            Map<String, List<Node>> convertedDestinations = new HashMap<>();
            for (Map.Entry<String, List<String>> destEntry : destPaths.entrySet()) {
                String endNode = destEntry.getKey();
                List<String> nodeNames = destEntry.getValue();

                List<Node> nodePath = nodeNames == null
                    ? List.of()
                    : nodeNames.stream()
                        .map(nodeLookup::get)
                        .filter(Objects::nonNull)
                        .collect(Collectors.toUnmodifiableList());
                convertedDestinations.put(endNode, nodePath);
            }
            convertedPaths.put(startNode, Collections.unmodifiableMap(convertedDestinations));
        }
        return Collections.unmodifiableMap(convertedPaths);
    }

    /**
     * Returns the fastest path (as a list of Node objects) from currentNode to requestedNode.
     * The lookup is O(1) thanks to our precomputed static map.
     *
     * @param currentNode The starting Node.
     * @param requestedNode The target Node.
     * @return A List of Nodes representing the fastest path, or an empty list if no path exists.
     */
    public static List<Node> getFastestPath(Node currentNode, Node requestedNode) {
        if (graphData == null || currentNode == null || requestedNode == null) {
            return Collections.emptyList();
        }

        Map<String, List<Node>> startMap = graphData.shortestPaths.get(currentNode.getName());
        if (startMap == null) {
            return Collections.emptyList();
        }
        return startMap.getOrDefault(requestedNode.getName(), Collections.emptyList());
    }

    public static Optional<Node> getNodeByName(String nodeName) {
        return Optional.ofNullable(nodeLookup.get(nodeName));
    }

    public static List<String> getSubsystemKeys() {
        return graphData == null ? DEFAULT_SUBSYSTEM_ORDER : graphData.subsystemKeys;
    }

    /**
     * Allows manual reloading of the graph if necessary.
     *
     * @param filePath The path to the new JSON file.
     */
    public static void reloadGraph(String filePath) {
        loadGraph(filePath);
    }
}