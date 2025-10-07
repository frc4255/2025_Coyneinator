package frc.robot;

import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.Node;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Centralised state tracking for high-level robot modes along with a convenience cache of preset
 * mechanism setpoints loaded from the AdvantageKit graph definition.
 */
public class StateManager {

  /** High-level task groupings for the robot. */
  public enum RobotStateMachine {
    Coral,
    Algae
  }

  /** Named scoring levels used by operator shortcuts. */
  public enum Positions {
    L1("L1 Score"),
    L2("L2 Align"),
    L3("L3 Dunk"),
    L4("L4 Dunk");

    private final String nodeName;

    Positions(String nodeName) {
      this.nodeName = nodeName;
    }

    String getNodeName() {
      return nodeName;
    }
  }

  private static final Map<Positions, double[]> POSITION_CACHE = new EnumMap<>(Positions.class);

  private static volatile RobotStateMachine currentState = RobotStateMachine.Coral;

  static {
    refreshCachedPositions();
  }

  /** Rebuilds the cached position arrays from the latest graph data. */
  public static synchronized void refreshCachedPositions() {
    POSITION_CACHE.clear();
    for (Positions position : Positions.values()) {
      POSITION_CACHE.put(position, loadSetpoints(position));
    }
  }

  private static double[] loadSetpoints(Positions position) {
    Optional<Node> node = GraphParser.getNodeByName(position.getNodeName());
    if (node.isEmpty()) {
      return new double[0];
    }

    List<String> subsystemOrder = GraphParser.getSubsystemKeys();
    double[] setpoints = new double[subsystemOrder.size()];
    for (int i = 0; i < subsystemOrder.size(); i++) {
      setpoints[i] =
          node.get()
              .getSetpoint(subsystemOrder.get(i))
              .orElse(0.0);
    }

    return setpoints;
  }

  /** Returns the currently selected high-level robot mode. */
  public static RobotStateMachine getCurrentState() {
    return currentState;
  }

  /** Sets the current high-level robot mode. */
  public static void setCurrentState(RobotStateMachine state) {
    currentState = Objects.requireNonNull(state);
  }

  /** Returns a copy of the cached mechanism setpoints for the requested level, if present. */
  public static double[] getCoordinate(Positions level) {
    double[] values = POSITION_CACHE.get(level);
    return values == null || values.length == 0 ? null : values.clone();
  }

  /** Toggles between coral and algae operating modes. */
  public void toggleRobotState() {
    currentState =
        currentState == RobotStateMachine.Coral
            ? RobotStateMachine.Algae
            : RobotStateMachine.Coral;
  }
}
