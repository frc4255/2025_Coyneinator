package frc.robot;

import java.util.Collections;
import java.util.List;
import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.Node;
import frc.robot.subsystems.GroundIntake;

/**
 * Manages ground intake pitch setpoints using the existing graph infrastructure. This keeps the
 * AdvantageKit logging format consistent while reducing the robot to only the swerve drive and
 * ground intake mechanisms.
 */
public class GroundIntakeGraphManager {
    private final GroundIntake groundIntake;

    private Node currentNode;
    private Node requestedNode;
    private List<Node> activePath = Collections.emptyList();
    private int pathIndex = 0;
    private boolean active = false;
    private double lastCommandedPitch = Constants.GroundIntake.DEFAULT_PITCH_RADIANS;
    private final double[] commandedSetpoints = new double[5];

    public GroundIntakeGraphManager(GroundIntake groundIntake) {
        this.groundIntake = Objects.requireNonNull(groundIntake);

        currentNode = GraphParser.getNodeByName("Idle");
        if (currentNode == null) {
            currentNode = GraphParser.getNodeByName("Start");
        }
        if (currentNode == null) {
            currentNode = new Node(
                "GroundIntakeDefault",
                new double[] {
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    Constants.GroundIntake.DEFAULT_PITCH_RADIANS
                }
            );
        }

        System.arraycopy(currentNode.getSetpoints(), 0, commandedSetpoints, 0, commandedSetpoints.length);
        lastCommandedPitch = commandedSetpoints[4];
        groundIntake.setPitchGoal(lastCommandedPitch);
    }

    /** Cancel any active path following. */
    public void setInactive() {
        active = false;
        activePath = Collections.singletonList(currentNode);
        pathIndex = 0;
        requestedNode = null;
    }

    /** Request a node by name from the graph database. */
    public void requestNodeByName(String nodeName) {
        Node node = GraphParser.getNodeByName(nodeName);
        if (node != null) {
            requestNode(node);
        }
    }

    /** Request a node directly. */
    public void requestNode(Node node) {
        if (node == null) {
            return;
        }

        requestedNode = node;
        if (currentNode == null) {
            currentNode = node;
        }

        List<Node> path = GraphParser.getFastestPath(currentNode, node);
        if (path == null || path.isEmpty()) {
            activePath = Collections.singletonList(node);
        } else {
            activePath = path;
        }

        pathIndex = 0;
        active = true;
        commandCurrentNode();
    }

    /** Should be called periodically (e.g., from {@code Robot.robotPeriodic}). */
    public void update() {
        Logger.recordOutput("GroundIntakeGraph/Active", active);
        Logger.recordOutput("GroundIntakeGraph/PathLength", activePath.size());
        Logger.recordOutput("GroundIntakeGraph/PathIndex", pathIndex);
        Logger.recordOutput(
            "GroundIntakeGraph/CurrentNode",
            currentNode != null ? currentNode.getName() : "None"
        );
        Logger.recordOutput(
            "GroundIntakeGraph/RequestedNode",
            requestedNode != null ? requestedNode.getName() : "None"
        );
        Logger.recordOutput("GroundIntakeGraph/CommandedSetpoints", commandedSetpoints);
        Logger.recordOutput("GroundIntakeGraph/CommandedPitchRadians", lastCommandedPitch);
        double pitchError = groundIntake.getPitchPosition() - lastCommandedPitch;
        Logger.recordOutput("GroundIntakeGraph/PitchErrorRadians", pitchError);

        if (!active || activePath.isEmpty() || pathIndex >= activePath.size()) {
            return;
        }

        Node target = activePath.get(pathIndex);
        double targetPitch = target.getSetpoints()[4];
        if (Math.abs(targetPitch - lastCommandedPitch) > 1e-6) {
            commandCurrentNode();
            targetPitch = lastCommandedPitch;
        }

        boolean atGoal = Math.abs(pitchError) <= Constants.GroundIntake.PITCH_POSITION_TOLERANCE_RADIANS
            && Math.abs(groundIntake.getPitchVelocity()) <= Constants.GroundIntake.PITCH_VELOCITY_TOLERANCE_RAD_PER_SEC;

        if (atGoal) {
            currentNode = target;
            pathIndex++;
            if (pathIndex >= activePath.size()) {
                active = false;
                activePath = Collections.singletonList(currentNode);
                pathIndex = 0;
            } else {
                commandCurrentNode();
            }
        }
    }

    private void commandCurrentNode() {
        if (activePath.isEmpty() || pathIndex >= activePath.size()) {
            return;
        }

        Node node = activePath.get(pathIndex);
        System.arraycopy(node.getSetpoints(), 0, commandedSetpoints, 0, commandedSetpoints.length);
        lastCommandedPitch = commandedSetpoints[4];
        groundIntake.setPitchGoal(lastCommandedPitch);
        Logger.recordOutput("GroundIntakeGraph/ActiveNode", node.getName());
    }
}
