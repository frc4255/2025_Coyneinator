package frc.lib.util.graph;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.OptionalDouble;

public class Node {
    private final String name;
    private final Map<String, Double> setpoints;

    public Node(String name, Map<String, Double> setpoints) {
        this.name = Objects.requireNonNull(name, "name");
        this.setpoints = Collections.unmodifiableMap(new LinkedHashMap<>(Objects.requireNonNull(setpoints, "setpoints")));
    }

    public String getName() {
        return name;
    }

    public Map<String, Double> getSetpoints() {
        return setpoints;
    }

    public OptionalDouble getSetpoint(String key) {
        Double value = setpoints.get(key);
        return value != null ? OptionalDouble.of(value) : OptionalDouble.empty();
    }
}
