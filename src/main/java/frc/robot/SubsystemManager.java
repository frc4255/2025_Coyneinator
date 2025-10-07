package frc.robot;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.Node;
import frc.robot.superstructure.Constraints;
import frc.robot.superstructure.ManipulatorProfile;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.DifferentialWrist;
import edu.wpi.first.wpilibj.Timer;

public class SubsystemManager {

    private final Pivot sPivot;
    private final Elevator sElevator;
    private final DifferentialWrist sWrist;
    
    private List<Node> path;
    private int currentIndex;
    private boolean active;

    private Node currentNode;
    private Node requestedNode;

    private Node lastNode;

    private boolean reactivation = false;

    private ManipulatorProfile currentProfile = ManipulatorProfile.TRANSIT;
    private double[] currentTolerance = currentProfile.toleranceCopy();
    private double minHoldTimeSeconds = currentProfile.defaultHoldTimeSeconds();
    private double holdStartTime = -1.0;
    private boolean holdSatisfied = true;
    private Node targetNode;
    private double[] targetSetpoints = new double[] {0, 0, 0, 0};
    private final double[] commandedSetpoints = new double[] {0, 0, 0, 0};

    public SubsystemManager(
            Pivot sPivot, Elevator sElevator, DifferentialWrist sWrist
        ) {

        this.sPivot = sPivot;
        this.sElevator = sElevator;
        this.sWrist = sWrist;
        
        this.active = false;
        this.currentIndex = 0;

        lastNode = new Node("Empty", new double[] {0, 0, 0, 0});

    }

    public void setInactive() {
        active = false;
    }
    public void requestNode(Node requestedNode) {
        requestNode(requestedNode, ManipulatorProfile.TRANSIT, null, ManipulatorProfile.TRANSIT.defaultHoldTimeSeconds());
    }

    public void requestNode(Node requestedNode, ManipulatorProfile profile, double[] tolerance, double minHoldTime) {

        if (requestedNode == null) {
            return;
        }

        if (currentNode == null) {
            currentNode = GraphParser.getNodeByName("Stow");
        }
        reactivation = true;
        lastNode = new Node("Empty", new double[] {0, 0, 0, 0});

        this.requestedNode = requestedNode;
        this.targetNode = requestedNode;
        this.targetSetpoints = requestedNode.getSetpoints();
        this.currentProfile = profile != null ? profile : ManipulatorProfile.TRANSIT;
        double[] fallbackTolerance = this.currentProfile.toleranceCopy();
        if (tolerance != null && tolerance.length == 4) {
            this.currentTolerance = Arrays.copyOf(tolerance, tolerance.length);
        } else {
            this.currentTolerance = fallbackTolerance;
        }
        this.minHoldTimeSeconds = minHoldTime >= 0.0 ? minHoldTime : this.currentProfile.defaultHoldTimeSeconds();
        this.holdStartTime = -1.0;
        this.holdSatisfied = this.minHoldTimeSeconds <= 0.0;

        this.currentIndex = 0;
        this.path = GraphParser.getFastestPath(currentNode, requestedNode);
        this.active = (path != null && !path.isEmpty());
    }

    /**
     * Call this method periodically (e.g., in Robot.periodic()).
     * It processes the current node in the path and commands each subsystem accordingly.
     */
    public void update() {
        
        if (path == null) {
            return;
        }

        double pivotPosition = sPivot.getPivotPosition();
        double elevatorPosition = sElevator.getElevatorPosition();
        double wristPitchPosition = sWrist.getPitchPosition();
        double wristRollPosition = sWrist.getRollPosition();

        double[] measurements = new double[] {pivotPosition, elevatorPosition, wristPitchPosition, wristRollPosition};

        if (!active || currentIndex >= path.size()) {
            evaluateHold(measurements);
            return;
        }

        currentNode = path.get(currentIndex);
        double[] setpoints = currentNode.getSetpoints();

        if (setpoints.length != 4) {
            setpoints = Arrays.copyOf(setpoints, 4);
        }

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

        double pivotGoal = Constraints.clampPivot(setpoints[0], pivotPosition, elevatorPosition);
        double elevatorGoal = Constraints.clampElevator(setpoints[1], elevatorPosition, pivotPosition, currentProfile);
        double wristPitchGoal = Constraints.clampPitch(setpoints[2], pivotPosition, elevatorPosition, currentProfile);
        double wristRollGoal = Constraints.clampRoll(setpoints[3]);

        commandedSetpoints[0] = pivotGoal;
        commandedSetpoints[1] = elevatorGoal;
        commandedSetpoints[2] = wristPitchGoal;
        commandedSetpoints[3] = wristRollGoal;

        if (reactivation || !currentNode.getName().equals(lastNode.getName())) {
            reactivation = false;
            sWrist.setActive();
            sElevator.setActive();
        }

        sPivot.setGoal(pivotGoal);
        sElevator.setGoal(elevatorGoal);
        sWrist.setGoals(wristPitchGoal, wristRollGoal);

        if (hasReachedTarget(setpoints, measurements)) {
            lastNode = currentNode;
            currentIndex++;
            if (currentIndex >= path.size()) {
                active = false;
                holdStartTime = -1.0;
                holdSatisfied = minHoldTimeSeconds <= 0.0;
            }
        }

        canAutoHome();

        Logger.recordOutput("CurrentNode", currentNode.getName());
        Logger.recordOutput("Manager/Active", active);
        Logger.recordOutput("Manager/HoldSatisfied", holdSatisfied);
    }

    /**
     * Checks if all subsystems have reached their target.
     * Assumes each subsystem has an atGoal() method that returns true when the target is reached.
     *
     * @return true if all subsystems are at their target, false otherwise.
     */
    public boolean hasReachedTarget(double[] setpoints, double[] measurements) {
        if (setpoints.length != 4 || measurements.length != 4) {
            return false;
        }

        for (int i = 0; i < 4; i++) {
            if (Math.abs(setpoints[i] - measurements[i]) > currentTolerance[i]) {
                return false;
            }
        }
        return true;
    }

    public boolean hasReachedGoal(String x) {
        if (currentNode.getName().equals(x) && hasReachedTarget() && currentNode != lastNode) {
            return true;
        } else {
            return false;
        }
    }

    public boolean canAutoHome() {
        if (currentNode.getName().equals("Stow") && hasReachedTarget() && currentNode != lastNode) {
            System.err.println("can Auto Home");
            return true;
        } else {
            return false;
        }
    }

    private boolean hasReachedTarget() {
        return !active && holdSatisfied;
    }

    public boolean isTargetSettled() {
        return !active && holdSatisfied;
    }

    private void evaluateHold(double[] measurements) {
        if (targetNode == null) {
            return;
        }

        if (!withinTolerance(targetSetpoints, measurements, currentTolerance)) {
            holdStartTime = -1.0;
            holdSatisfied = minHoldTimeSeconds <= 0.0;
            return;
        }

        if (holdSatisfied) {
            return;
        }

        if (holdStartTime < 0.0) {
            holdStartTime = Timer.getFPGATimestamp();
        }

        double elapsed = Timer.getFPGATimestamp() - holdStartTime;
        if (elapsed >= minHoldTimeSeconds) {
            holdSatisfied = true;
        }
    }

    private boolean withinTolerance(double[] setpoints, double[] measurements, double[] tolerance) {
        if (setpoints.length != 4 || measurements.length != 4) {
            return false;
        }

        for (int i = 0; i < 4; i++) {
            if (Math.abs(setpoints[i] - measurements[i]) > tolerance[i]) {
                return false;
            }
        }
        return true;
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
