package frc.lib.util.graph;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.io.InputStream;
import java.util.*;

public class GraphParser {

    // A lookup table for Node objects (for O(1) access)
    private static final Map<String, Node> nodeLookup = new HashMap<>();

    // This is the class we use in our code.
    public static class GraphData {
        public List<Node> nodes;
        public List<Edge> edges;
        public Map<String, Map<String, List<Node>>> shortestPaths;
    }

    // This class is used only for JSON deserialization.
    public static class GraphDataRaw {
        public List<Node> nodes;
        public List<Edge> edges;
        public Map<String, Map<String, List<String>>> shortestPaths;
    }
    
    private static GraphData graphData;

    // Static initializer: load the graph once when the class is first referenced.
    static {
        loadGraph("/graph_data.json");
    }

    public static void funny() {
    }
    /**
     * Loads the graph from the JSON file. It first deserializes the JSON into a GraphDataRaw
     * (which expects shortestPaths as lists of String) and then converts it into our GraphData
     * with shortestPaths as lists of Node.
     *
     * @param filePath The path to the JSON file.
     */
    private static void loadGraph(String filePath) {
        ObjectMapper mapper = new ObjectMapper();
        File deployFile = new File(Filesystem.getDeployDirectory(), "graph_data.json");
        try (InputStream input = openGraphStream(deployFile)) {
            if (input == null) {
                System.err.println("Failed to locate graph_data.json for parsing.");
                return;
            }
            nodeLookup.clear();
            // First, deserialize the raw JSON data.
            GraphDataRaw raw = mapper.readValue(input, GraphDataRaw.class);

            // Create our GraphData instance.
            graphData = new GraphData();
            graphData.nodes = raw.nodes;
            graphData.edges = raw.edges;
            // Build the node lookup map for O(1) access.
            for (Node node : graphData.nodes) {
                nodeLookup.put(node.getName(), node);
            }

            // Convert the raw shortestPaths (List<String>) into List<Node>
            graphData.shortestPaths = convertShortestPaths(raw.shortestPaths);
            validateGraph();
        } catch (Exception e) {
            e.printStackTrace();
        }

        System.out.println(nodeLookup);
    }

    private static InputStream openGraphStream(File deployFile) {
        try {
            if (deployFile.exists()) {
                System.out.println("Loaded graph_data.json from deploy directory: " + deployFile.getAbsolutePath());
                return Files.newInputStream(deployFile.toPath());
            }

            Path projectPath = Path.of("src", "main", "deploy", "graph_data.json");
            if (Files.exists(projectPath)) {
                System.out.println("Loaded graph_data.json from project deploy folder: " + projectPath.toAbsolutePath());
                return Files.newInputStream(projectPath);
            }

            InputStream resourceStream = GraphParser.class.getResourceAsStream("/graph_data.json");
            if (resourceStream != null) {
                System.out.println("Loaded graph_data.json from classpath resource.");
            }
            return resourceStream;
        } catch (IOException ioException) {
            ioException.printStackTrace();
            return null;
        }
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
        Map<String, Map<String, List<Node>>> convertedPaths = new HashMap<>();

        if (stringPaths == null) {
            return convertedPaths;
        }

        for (Map.Entry<String, Map<String, List<String>>> entry : stringPaths.entrySet()) {
            String startNode = entry.getKey();
            Map<String, List<String>> destPaths = entry.getValue();

            Map<String, List<Node>> convertedDestinations = new HashMap<>();
            for (Map.Entry<String, List<String>> destEntry : destPaths.entrySet()) {
                String endNode = destEntry.getKey();
                List<String> nodeNames = destEntry.getValue();

                List<Node> nodePath = new ArrayList<>();
                for (String nodeName : nodeNames) {
                    Node node = nodeLookup.get(nodeName);
                    if (node != null) {
                        nodePath.add(node);
                    }
                }
                convertedDestinations.put(endNode, nodePath);
            }
            convertedPaths.put(startNode, convertedDestinations);
        }
        return convertedPaths;
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
        if (graphData == null || graphData.shortestPaths == null ||
            !graphData.shortestPaths.containsKey(currentNode.getName())) {
            return Collections.emptyList();
        }
        return graphData.shortestPaths.getOrDefault(currentNode.getName(), Collections.emptyMap())
                                      .getOrDefault(requestedNode.getName(), Collections.emptyList());
    }

    public static Node getNodeByName(String nodeName) {
        if (nodeName == null) {
            return null;
        }
        return nodeLookup.get(nodeName.trim());
    }
    /**
     * Allows manual reloading of the graph if necessary.
     *
     * @param filePath The path to the new JSON file.
     */
    public static void reloadGraph(String filePath) {
        loadGraph(filePath);
    }

    private static void validateGraph() {
        if (graphData == null) {
            return;
        }

        for (Edge edge : graphData.edges) {
            if (!nodeLookup.containsKey(edge.getStart())) {
                System.err.println("Graph edge start missing node: " + edge.getStart());
            }
            if (!nodeLookup.containsKey(edge.getEnd())) {
                System.err.println("Graph edge end missing node: " + edge.getEnd());
            }
        }

        if (graphData.shortestPaths != null) {
            for (Map.Entry<String, Map<String, List<Node>>> entry : graphData.shortestPaths.entrySet()) {
                if (!nodeLookup.containsKey(entry.getKey())) {
                    System.err.println("Shortest path source missing node: " + entry.getKey());
                }
                for (Map.Entry<String, List<Node>> targetEntry : entry.getValue().entrySet()) {
                    if (!nodeLookup.containsKey(targetEntry.getKey())) {
                        System.err.println("Shortest path destination missing node: " + targetEntry.getKey());
                    }
                }
            }
        }
    }
}
