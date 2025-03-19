package frc.lib.util.graph;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class Edge {
    private String start;
    private String end;

    public Edge() {

    }
    @JsonCreator
    public Edge(@JsonProperty("start") String start, @JsonProperty("end") String end) {
        this.start = start;
        this.end = end;
    }

    public String getStart() {
        return start;
    }

    public String getEnd() {
        return end;
    }
}
