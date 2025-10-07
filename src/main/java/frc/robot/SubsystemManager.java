package frc.robot;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.Node;
import frc.robot.subsystems.CoordinatedSubsystem;

public class SubsystemManager {

    public static final String PIVOT_KEY = "pivot";
    public static final String ELEVATOR_KEY = "elevator";
    public static final String WRIST_PITCH_KEY = "wristPitch";
    public static final String WRIST_ROLL_KEY = "wristRoll";

    private final Map<String, CoordinatedSubsystem> subsystemMap = new HashMap<>();
    private final Set<String> activeSubsystemKeys = new HashSet<>();

    private List<Node> path = List.of();
    private int currentIndex;
    private boolean active;

    private Node currentNode;
    private Node requestedNode;

    private Node lastNode;

    private boolean reactivation = false;

    public SubsystemManager(CoordinatedSubsystem... subsystems) {
        registerSubsystems(List.of(subsystems));
    }

    public SubsystemManager(Collection<CoordinatedSubsystem> subsystems) {
        registerSubsystems(subsystems);
    }

    public void registerSubsystem(CoordinatedSubsystem subsystem) {
        if (subsystem != null) {
            subsystemMap.put(subsystem.key(), subsystem);
        }
    }

    public void registerSubsystems(Collection<CoordinatedSubsystem> subsystems) {
        if (subsystems != null) {
            subsystems.forEach(this::registerSubsystem);
        }
    }

    public void setCurrentNode(Node node) {
        this.currentNode = node;
    }

    public void setCurrentNode(String nodeName) {
        GraphParser.getNodeByName(nodeName).ifPresent(this::setCurrentNode);
    }

    public Optional<Node> getCurrentNode() {
        return Optional.ofNullable(currentNode);
    }

    public Optional<Node> getRequestedNode() {
        return Optional.ofNullable(requestedNode);
    }

    public void requestNode(String nodeName) {
        GraphParser.getNodeByName(nodeName).ifPresent(this::requestNode);
    }

    public void setInactive() {
        active = false;
    }

    public boolean hasReachedGoal(String nodeName) {
        if (nodeName == null || nodeName.isBlank()) {
            return false;
        }

        return !active
            && currentNode != null
            && currentNode.getName().equals(nodeName);
    }

    public boolean hasReachedGoal(Node node) {
        return node != null && hasReachedGoal(node.getName());
    }
    public void requestNode(Node requestedNode) {
        if (requestedNode == null) {
            return;
        }

        this.requestedNode = requestedNode;
        this.currentIndex = 0;

        Node startNode = currentNode != null ? currentNode : requestedNode;
        List<Node> computedPath = GraphParser.getFastestPath(startNode, requestedNode);
        if (computedPath == null || computedPath.isEmpty()) {
            this.path = List.of();
            this.active = false;
            if (startNode == requestedNode) {
                this.currentNode = requestedNode;
            }
            return;
        }

        this.path = computedPath;
        this.active = true;
    }

    /**
     * Call this method periodically (e.g., in Robot.periodic()).
     * It processes the current node in the path and commands each subsystem accordingly.
     */
    public void update() {

        if (!active || path.isEmpty() || currentIndex >= path.size()) {
            return;
        }

        Node node = path.get(currentIndex);

        if (currentIndex == 0 && currentNode != null && node.getName().equals(currentNode.getName())) {
            advanceToNext(node);
            return;
        }

        activeSubsystemKeys.clear();
        node.getSetpoints().forEach((key, value) -> {
            CoordinatedSubsystem subsystem = subsystemMap.get(key);
            if (subsystem != null) {
                subsystem.setGoal(value);
                activeSubsystemKeys.add(key);
            }
        });

        if (activeSubsystemKeys.isEmpty() || hasReachedTarget()) {
            advanceToNext(node);
        }
    }

    private void advanceToNext(Node processedNode) {
        currentNode = processedNode;
        currentIndex++;
        if (currentIndex >= path.size()) {
            active = false;
            currentNode = requestedNode;
        }
    }

    /**
     * Checks if all subsystems have reached their target.
     */
    private boolean hasReachedTarget() {
        for (String key : activeSubsystemKeys) {
            CoordinatedSubsystem subsystem = subsystemMap.get(key);
            if (subsystem != null && !subsystem.atGoal()) {
                return false;
            }
        }
        return true;
    }

    public void cancel() {
        active = false;
        path = List.of();
        currentIndex = 0;
        activeSubsystemKeys.clear();
    }

    public Set<String> getRegisteredSubsystemKeys() {
        return Collections.unmodifiableSet(subsystemMap.keySet());
    }

    public boolean isActive() {
        return active;
    }

    public boolean canAutoHome() {
        return !active;
    }
}
