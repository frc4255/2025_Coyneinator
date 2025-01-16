package frc.robot;

import java.lang.Thread.State;
import java.util.HashMap;

public class StateManager {

    public enum RobotStateMachine {
        Coral,
        Algae
    }

    private RobotStateMachine currentState = RobotStateMachine.Coral;

    public enum Positions {
        L1,
        L2,
        L3,
        L4,
        NET,
        HP,
        INTAKE
    }

    
    private final HashMap<Positions, Double[]> coral = new HashMap<>();
    private final HashMap<Positions, Double[]> algae = new HashMap<>();

    private HashMap<Positions, Double[]> currentCoordinateBase = new HashMap<>();

    public StateManager() {

        coral.put(Positions.L1, new Double[] {0.0, 0.0}); //TODO Tune this
        coral.put(Positions.L2, new Double[] {0.0, 0.0}); //TODO Tune this
        coral.put(Positions.L3, new Double[] {0.0, 0.0}); //TODO Tune this
        coral.put(Positions.L4, new Double[] {0.0, 0.0}); //TODO Tune this
        coral.put(Positions.NET, new Double[] {0.0, 0.0}); //TODO Tune this
        coral.put(Positions.HP, new Double[] {0.0, 0.0}); //TODO Tune this

        algae.put(Positions.L1, new Double[] {0.0, 0.0}); //TODO Tune this
        algae.put(Positions.L2, new Double[] {0.0, 0.0}); //TODO Tune this
        algae.put(Positions.L3, new Double[] {0.0, 0.0}); //TODO Tune this
        algae.put(Positions.L4, new Double[] {0.0, 0.0}); //TODO Tune this
        algae.put(Positions.NET, new Double[] {0.0, 0.0}); //TODO Tune this
        algae.put(Positions.HP, new Double[] {0.0, 0.0}); //TODO Tune this
        algae.put(Positions.INTAKE, new Double[] {0.0, 0.0}); //TODO Tune this

    }

    public Double[] getCoordinate(Positions requested) {

        if (getCurrentState() == RobotStateMachine.Algae) {
            currentCoordinateBase = algae;
        } else {
            currentCoordinateBase = coral;
        }

        return currentCoordinateBase.get(requested);
    }

    public RobotStateMachine getCurrentState() {
        return currentState;
    }

    public void setRobotState(RobotStateMachine newState) {
        currentState = newState;
    }
}