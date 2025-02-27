package frc.lib.util.graph;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.util.List;
import java.util.HashMap;
import java.util.Map;

public class GraphParser {
    public static class GraphData {
        public List<Node> nodes;
        public List<Edge> edges;
    }

    public static GraphData parse(String filePath) {
        ObjectMapper mapper = new ObjectMapper();
        GraphData data = null;
        try {
            data = mapper.readValue(new File("./all_paths.json"), GraphData.class);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return data;
    }

    public static Map<String, Node> buildNodeMap(List<Node> nodes) {
        Map<String, Node> map = new HashMap<>();
        for (Node node : nodes) {
            map.put(node.getName(), node);
        }
        return map;
    }
}