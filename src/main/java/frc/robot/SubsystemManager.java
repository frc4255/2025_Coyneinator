package frc.robot;

import java.util.List;

import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.Node;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;

public class SubsystemManager {

    private final Pivot sPivot;
    private final Elevator sElevator;
    private final WristPitch sWristPitch;
    private final WristRoll sWristRoll;
    
    private List<Node> path;
    private int currentIndex;
    private boolean active;

    private Node currentNode;
    private Node requestedNode;

    private Node lastNode;

    public SubsystemManager(
            Pivot sPivot, Elevator sElevator, WristPitch sWristPitch, WristRoll sWristRoll
        ) {

        this.sPivot = sPivot;
        this.sElevator = sElevator;
        this.sWristPitch = sWristPitch;
        this.sWristRoll = sWristRoll;
        
        this.active = false;
        this.currentIndex = 0;

        double[] test = new double[]{0,0,0,0};
    }

    public void setInactive() {
        active = false;
    }
    public void requestNode(Node requestedNode) {

        if (currentNode == null) {
            currentNode = GraphParser.getNodeByName("Stow");
        }
        this.requestedNode = requestedNode;
        this.currentIndex = 0;

        System.out.println(GraphParser.getFastestPath(currentNode, requestedNode));

        this.path = GraphParser.getFastestPath(currentNode, requestedNode);
        this.active = (path != null && !path.isEmpty());
    }

    /**
     * Call this method periodically (e.g., in Robot.periodic()).
     * It processes the current node in the path and commands each subsystem accordingly.
     */
    public void update() {
        
        if (!active || path == null || currentIndex >= path.size()) {
            return;
        }

        Node currentNode = path.get(currentIndex);
        double[] setpoints = currentNode.getSetpoints();

        /*  Code to automatically go to reef align, can be added back based on driver feedback

        Not completed, isWithinReefZone() likely requires Swerve subsystem.
        if (currentNode.getName().equalsIgnoreCase("Stow")) {
            if (isWithinReefZone()) {
                setpoints = REEF_ALIGN_SETPOINTS;
            } else {
                setpoints = STOW_SETPOINTS;
            }
        }
       */

       if (currentNode != lastNode) {
        sWristPitch.setActive();
        sWristRoll.setActive();


        sPivot.setGoal(setpoints[0]);
        sElevator.setGoal(setpoints[1]);
        sWristPitch.setGoal(setpoints[2]);
        sWristRoll.setGoal(setpoints[3]);

       }
        // If all subsystems have reached their targets, move to the next node.
        if (hasReachedTarget()) {
            currentIndex++;
            if (currentIndex >= path.size()) {
                active = false;
            }
            System.out.println("IT FUCKING WORKS");
        }
    }

    /**
     * Checks if all subsystems have reached their target.
     * Assumes each subsystem has an atGoal() method that returns true when the target is reached.
     *
     * @return true if all subsystems are at their target, false otherwise.
     */
    private boolean hasReachedTarget() {
        return sPivot.atGoal() && sElevator.atGoal() &&
               sWristPitch.atGoal() && sWristRoll.atGoal();
    }

    /**
     * Returns whether the manager is actively processing a path.
     *
     * @return true if active, false otherwise.
     */
    public boolean isActive() {
        return active;
    } 
}
