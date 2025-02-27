package frc.lib.util.graph;

public class Edge {
    private String start;
    private String end;
    private String description;

    public Edge(String start, String end, String description) {
        this.start = start;
        this.end = end;
        this.description = description;
    }

    public String getStart() {
        return start;
    }

    public String getEnd() {
        return end;
    }

    public String getDescription() {
        return description;
    }   
}
