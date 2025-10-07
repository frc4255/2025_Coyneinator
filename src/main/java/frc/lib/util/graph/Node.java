package frc.lib.util.graph;

import java.util.Arrays;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class Node {
    private final String name;
    private final double[] setpoints; // length 4
    
    /* Order or setpoints is based on progression of DOFs 
     * Index 0 = Pivot
     * Index 1 = Elevator Length
     * Index 2 = Wrist Pitch
     * Index 3 = Wrist Roll
    */

    @JsonCreator
    public Node(@JsonProperty("name") String name, @JsonProperty("setpoints") double[] setpoints) {
        this.name = name != null ? name.trim() : "";
        double[] values = setpoints != null ? setpoints : new double[0];
        if (values.length < 4) {
            throw new IllegalArgumentException("Node setpoints must contain at least 4 values for " + this.name);
        }
        if (values.length > 4) {
            values = Arrays.copyOf(values, 4);
        }
        this.setpoints = Arrays.copyOf(values, values.length);
    }

    public String getName() {
        return name;
    }

    public double[] getSetpoints() {
        return Arrays.copyOf(setpoints, setpoints.length);
    }
}
