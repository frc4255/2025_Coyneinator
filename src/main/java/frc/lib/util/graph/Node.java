package frc.lib.util.graph;

public class Node {
    private String name;
    private double[] setpoints; // length 4
    
    /* Order or setpoints is based on progression of DOFs 
     * Index 0 = Pivot
     * Index 1 = Elevator Length
     * Index 2 = Wrist Pitch
     * Index 3 = Wrist Roll
    */

    public Node(String name, double[] setpoints) {
        this.name = name;
        this.setpoints = setpoints;
    }

    public String getName() {
        return name;
    }

    public double[] getSetpoints() {
        return setpoints;
    }
}
