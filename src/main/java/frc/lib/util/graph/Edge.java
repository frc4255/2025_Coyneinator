package frc.lib.util.graph;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class Edge {
    private final String start;
    private final String end;

    public Edge() {
        this("", "");
    }
    @JsonCreator
    public Edge(@JsonProperty("start") String start, @JsonProperty("end") String end) {
        this.start = start != null ? start.trim() : "";
        this.end = end != null ? end.trim() : "";
    }

    public String getStart() {
        return start;
    }

    public String getEnd() {
        return end;
    }
}
