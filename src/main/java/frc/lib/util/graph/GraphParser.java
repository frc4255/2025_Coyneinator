package frc.lib.util.graph;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.InputStream;
import java.util.*;

public class GraphParser {

    // A lookup table for Node objects (for O(1) access)
    private static Map<String, Node> nodeLookup = new HashMap<>();

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
        try {
            // First, deserialize the raw JSON data.
            GraphDataRaw raw = mapper.readValue(deployFile, GraphDataRaw.class);

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
        } catch (Exception e) {
            e.printStackTrace();
        }

        System.out.println(nodeLookup);
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
        return nodeLookup.get(nodeName);
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