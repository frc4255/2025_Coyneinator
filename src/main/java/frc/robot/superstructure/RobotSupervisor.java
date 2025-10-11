package frc.robot.superstructure;

import java.util.Arrays;
import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.Node;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.GroundIntake;

/**
 * High-level coordinator for superstructure intents.
 *
 * <p>The supervisor owns:
 * <ul>
 *   <li>Requesting nodes through {@link SubsystemManager}</li>
 *   <li>Managing tolerance/hold settings for different kinds of motion</li>
 *   <li>Tracking game-piece state and simple mode flags (algae/climb)</li>
 *   <li>Providing helpers for commands to run intake / end-effector actions</li>
 * </ul>
 *
 * <p>Driver automation will be implemented in commands that call these helpers. No plan/step
 * abstraction remains; commands choose the node and motion settings explicitly.</p>
 */
public final class RobotSupervisor {
    public enum Mode {
        CORAL,
        ALGAE,
        CLIMB
    }

    public enum ScoreLevel {
        L1,
        L2,
        L3,
        L4
    }

    private static final double[] TRANSIT_TOLERANCE = {0.06, 0.04, 0.08, 0.12, 0.10};
    private static final double[] SCORE_TOLERANCE = {0.04, 0.03, 0.05, 0.08, 0.06};
    private static final double[] CLIMB_TOLERANCE = {0.05, 0.04, 0.06, 0.10, 0.08};

    private static final double TRANSIT_HOLD_SECONDS = 0.0;
    private static final double SCORE_HOLD_SECONDS = 0.12;
    private static final double CLIMB_HOLD_SECONDS = 0.10;

    private final SubsystemManager manager;
    private final EndEffector endEffector;
    private final GroundIntake groundIntake;
    private final GamePieceState pieceState = new GamePieceState();
    private final PieceSensors sensors;

    private String lastRequestedNode = "Idle";
    private double[] lastRequestedTolerance = Arrays.copyOf(TRANSIT_TOLERANCE, TRANSIT_TOLERANCE.length);
    private double lastRequestedHold = TRANSIT_HOLD_SECONDS;
    private boolean lastRequestUsedClimbMode = false;

    private boolean algaeMode = false;
    private boolean climbMode = false;
    private Mode mode = Mode.CORAL;

    private boolean lastEndEffectorOccupied = false;
    private double lastAlgaeAcquisitionTimestamp = -10.0;
    private static final double ALGAE_CONFIDENCE_WINDOW_SECONDS = 1.0;

    public RobotSupervisor(
            SubsystemManager manager,
            GroundIntake groundIntake,
            EndEffector endEffector,
            PieceSensors sensors
    ) {
        this.manager = Objects.requireNonNull(manager);
        this.groundIntake = Objects.requireNonNull(groundIntake);
        this.endEffector = Objects.requireNonNull(endEffector);
        this.sensors = Objects.requireNonNull(sensors);
    }

    public void periodic() {
        pieceState.updateFromSensors(sensors);
        if (pieceState.isAlgaeInIntake() || pieceState.isAlgaeInEndEffector()) {
            algaeMode = true;
        }

        boolean endEffectorSensor = sensors.hasAlgaeEndEffectorSensor()
                && sensors.algaeDetectedAtEndEffector();
        updatePossessionHeuristics(endEffectorSensor);

        if (climbMode) {
            mode = Mode.CLIMB;
        } else if (algaeMode) {
            mode = Mode.ALGAE;
        } else {
            mode = Mode.CORAL;
        }

        Logger.recordOutput("Supervisor/RequestedNode", lastRequestedNode);
        Logger.recordOutput("Supervisor/NodeSettled", manager.isTargetSettled());
        Logger.recordOutput("Supervisor/ClimbMode", climbMode);
        Logger.recordOutput("Supervisor/AlgaeMode", algaeMode);
        Logger.recordOutput("Supervisor/Mode", mode.name());
        Logger.recordOutput("Supervisor/EndEffectorSensor", endEffectorSensor);
        Logger.recordOutput("Supervisor/CoralInIntake", pieceState.isCoralInIntake());
        Logger.recordOutput("Supervisor/CoralInWrist", pieceState.isCoralInWrist());
        Logger.recordOutput("Supervisor/AlgaeInIntake", pieceState.isAlgaeInIntake());
        Logger.recordOutput("Supervisor/AlgaeInEndEffector", pieceState.isAlgaeInEndEffector());
    }

    /** Request a node using transit tolerances. */
    public void goToTransit(String nodeName) {
        goToNode(nodeName, TRANSIT_TOLERANCE, TRANSIT_HOLD_SECONDS, false);
    }

    /** Request a node using the tighter scoring tolerances. */
    public void goToScorePose(String nodeName) {
        goToNode(nodeName, SCORE_TOLERANCE, SCORE_HOLD_SECONDS, false);
    }

    /** Request a node intended for climb operations. */
    public void goToClimbPose(String nodeName) {
        climbMode = true;
        mode = Mode.CLIMB;
        goToNode(nodeName, CLIMB_TOLERANCE, CLIMB_HOLD_SECONDS, true);
    }

    /**
     * Request a node with explicit motion settings.
     *
     * @param nodeName          Target node name
     * @param tolerance         Five-element tolerance array (pivot, elevator, wrist pitch, wrist roll, intake)
     * @param minHoldSeconds    Minimum time the position must remain within tolerance
     * @param climbConstraints  Whether to apply climb safety constraints for elevator/pitch
     */
    public void goToNode(String nodeName, double[] tolerance, double minHoldSeconds, boolean climbConstraints) {
        Node node = GraphParser.getNodeByName(nodeName);
        if (node == null) {
            Logger.recordOutput("Supervisor/UnknownNode", nodeName);
            return;
        }

        double[] toleranceCopy = tolerance != null && tolerance.length == 5
                ? Arrays.copyOf(tolerance, tolerance.length)
                : Arrays.copyOf(TRANSIT_TOLERANCE, TRANSIT_TOLERANCE.length);

        manager.requestNode(node, toleranceCopy, minHoldSeconds, climbConstraints);
        lastRequestedNode = nodeName;
        lastRequestedTolerance = toleranceCopy;
        lastRequestedHold = Math.max(minHoldSeconds, 0.0);
        lastRequestUsedClimbMode = climbConstraints;

        Logger.recordOutput("Supervisor/LastTolerance", lastRequestedTolerance);
        Logger.recordOutput("Supervisor/LastHoldSeconds", lastRequestedHold);
    }

    public boolean isTargetSettled() {
        return manager.isTargetSettled();
    }

    public void requestIdle() {
        climbMode = false;
        algaeMode = false;
        mode = Mode.CORAL;
        goToTransit("Idle");
    }

    public void setAlgaeMode(boolean enabled) {
        algaeMode = enabled;
        if (enabled) {
            mode = Mode.ALGAE;
        } else if (!climbMode) {
            mode = Mode.CORAL;
        }
    }

    public boolean isAlgaeMode() {
        return algaeMode;
    }

    public boolean isClimbMode() {
        return climbMode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode newMode) {
        mode = newMode;
        if (newMode != Mode.CLIMB) {
            climbMode = false;
        }
        algaeMode = newMode == Mode.ALGAE;
    }

    public GamePieceState getPieceState() {
        return pieceState;
    }

    public void markCoralInWrist(boolean value) {
        pieceState.setCoralInWrist(value);
    }

    public void markCoralInIntake(boolean value) {
        pieceState.setCoralInIntake(value);
    }

    public void markAlgaeInEndEffector(boolean value) {
        pieceState.setAlgaeInEndEffector(value);
    }

    public void markAlgaeInIntake(boolean value) {
        pieceState.setAlgaeInIntake(value);
    }

    public void runEndEffector(double volts) {
        endEffector.setDutyCycle(volts);
    }

    public void stopEndEffector() {
        endEffector.stop();
    }

    public void runGroundIntakeRoller(double volts) {
        groundIntake.setRollerVolts(volts);
    }

    public void stopGroundIntake() {
        groundIntake.stop();
    }

    public void clearAutomation() {
        manager.setInactive();
        stopEndEffector();
        stopGroundIntake();
        pieceState.clearAll();
        algaeMode = false;
        climbMode = false;
        mode = Mode.CORAL;
        lastEndEffectorOccupied = false;
        lastAlgaeAcquisitionTimestamp = -10.0;
        lastRequestedNode = "Idle";
        lastRequestedTolerance = Arrays.copyOf(TRANSIT_TOLERANCE, TRANSIT_TOLERANCE.length);
        lastRequestedHold = TRANSIT_HOLD_SECONDS;
        lastRequestUsedClimbMode = false;
    }

    public String getLastRequestedNode() {
        return lastRequestedNode;
    }

    public double[] getLastRequestedTolerance() {
        return Arrays.copyOf(lastRequestedTolerance, lastRequestedTolerance.length);
    }

    public double getLastRequestedHoldSeconds() {
        return lastRequestedHold;
    }

    public boolean lastRequestUsedClimbMode() {
        return lastRequestUsedClimbMode;
    }

    public void recordAlgaeAcquisition() {
        lastAlgaeAcquisitionTimestamp = Timer.getFPGATimestamp();
        setAlgaeMode(true);
    }

    public void recordCoralAcquisition() {
        setMode(Mode.CORAL);
    }

    private void updatePossessionHeuristics(boolean endEffectorSensor) {
        if (!sensors.hasAlgaeEndEffectorSensor()) {
            return;
        }

        if (endEffectorSensor && !lastEndEffectorOccupied) {
            boolean assumeAlgae = mode == Mode.ALGAE
                    || pieceState.isAlgaeInIntake()
                    || (Timer.getFPGATimestamp() - lastAlgaeAcquisitionTimestamp) < ALGAE_CONFIDENCE_WINDOW_SECONDS;

            if (assumeAlgae) {
                pieceState.setAlgaeInEndEffector(true);
                if (!sensors.hasCoralWristSensor()) {
                    pieceState.setCoralInWrist(false);
                }
            } else {
                pieceState.setAlgaeInEndEffector(false);
                if (!sensors.hasCoralWristSensor()) {
                    pieceState.setCoralInWrist(true);
                }
            }
        } else if (!endEffectorSensor && lastEndEffectorOccupied) {
            pieceState.setAlgaeInEndEffector(false);
            if (!sensors.hasCoralWristSensor()) {
                pieceState.setCoralInWrist(false);
            }
        }

        lastEndEffectorOccupied = endEffectorSensor;
    }
}




