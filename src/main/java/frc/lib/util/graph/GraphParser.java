package frc.lib.util.graph;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.util.List;
import java.util.Map;

public class GraphParser {

    private GraphData graphData;

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
        } catch (Exception e) {
            e.printStackTrace();
            this.graphData = null; // Set to null if parsing fails.
        }
    }

    public List<String> getFastestPath(String currentNodeName, String requestedNodeName) {
        if (graphData != null && graphData.shortestPaths.containsKey(currentNodeName)) {
            return graphData.shortestPaths.get(currentNodeName).get(requestedNodeName);
        }
        return null; // No valid path found. TODO: Null pointer exception bad
    }

}