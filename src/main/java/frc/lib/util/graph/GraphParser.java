package frc.lib.util.graph;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class GraphParser {

    private GraphData graphData;
    private Map<String, Node> nodeLookup; // O(1) name-to-Node lookup

    public static class GraphData {
        public List<Node> nodes;
        public List<Edge> edges;
        public Map<String, Map<String, List<String>>> shortestPaths;
    }

    /**
     * Parse the JSON file at the given file path.
     *
     * @param filePath the path to the JSON file
     * @return a GraphData object containing nodes and edges, or null if parsing fails.
     */

    public GraphParser() {
        ObjectMapper mapper = new ObjectMapper();
        try {
            this.graphData = mapper.readValue(new File("./graph_data.json"), GraphData.class);
            buildNodeLookup();
        } catch (Exception e) {
            e.printStackTrace();
            this.graphData = null; // Set to null if parsing fails.
        }
    }

     /**
     * Builds a HashMap for O(1) node lookup.
     */
    private void buildNodeLookup() {
        nodeLookup = new HashMap<>();
        if (graphData != null) {
            for (Node node : graphData.nodes) {
                nodeLookup.put(node.getName(), node);
            }
        }
    }

    /**
     * Returns the list of nodes in the graph.
     * @return List of nodes, or empty list if parsing failed.
     */
    public List<Node> getNodes() {
        return graphData != null ? graphData.nodes : Collections.emptyList();
    }

    /**
     * Returns the list of edges in the graph.
     * @return List of edges, or empty list if parsing failed.
     */
    public List<Edge> getEdges() {
        return graphData != null ? graphData.edges : Collections.emptyList();
    }

    /**
     * Returns the shortest path from currentNode to requestedNode as a List of Nodes.
     * This method is O(1) due to HashMap lookups.
     *
     * @param currentNodeName The starting node name.
     * @param requestedNodeName The target node name.
     * @return List of Nodes representing the shortest path, or empty list if no path exists.
     */
    public List<Node> getFastestPath(String currentNodeName, String requestedNodeName) {
        if (graphData == null || !graphData.shortestPaths.containsKey(currentNodeName)) {
            return Collections.emptyList();
        }

        List<String> nodeNames = graphData.shortestPaths.get(currentNodeName).get(requestedNodeName);
        if (nodeNames == null) {
            return Collections.emptyList();
        }

        // Convert node names to Node objects using O(1) lookup
        List<Node> nodePath = new ArrayList<>();
        for (String nodeName : nodeNames) {
            Node node = nodeLookup.get(nodeName);
            if (node != null) {
                nodePath.add(node);
            }
        }

        return nodePath;
    }

}