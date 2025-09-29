package frc.lib.util.graph;

import java.util.Objects;
import java.util.Optional;

public class Edge {
    private final String start;
    private final String end;
    private final String description;

    public Edge(String start, String end) {
        this(start, end, "");
    }

    public Edge(String start, String end, String description) {
        this.start = Objects.requireNonNull(start, "start");
        this.end = Objects.requireNonNull(end, "end");
        this.description = description == null ? "" : description;
    }

    public String getStart() {
        return start;
    }

    public String getEnd() {
        return end;
    }

    public Optional<String> getDescription() {
        return description.isEmpty() ? Optional.empty() : Optional.of(description);
    }
}
