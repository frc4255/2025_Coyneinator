package frc.robot;

import java.util.HashMap;

public class StateManager {

    public enum RobotStateMachine {
        Coral,
        Algae
    }

    private static RobotStateMachine currentState = RobotStateMachine.Coral;

    public enum Positions {
        L1,
        L2,
        L3,
        L4,
        NET,
        HP,
        INTAKE,
        STOW
    }

    
    private static final HashMap<Positions, double[]> coral = createCoralCoordinateMap();
    private static final HashMap<Positions, double[]> algae = createAlgaeCoordinateMap();

    private static HashMap<Positions, double[]> currentCoordinateBase = coral;

    public StateManager() {}

    private static HashMap<Positions, double[]> createCoralCoordinateMap() {
        HashMap<Positions, double[]> coralCoordinates = new HashMap<>();

        coralCoordinates.put(Positions.L1, new double[] {0.0, 0.0}); //TODO Tune this
        coralCoordinates.put(Positions.L2, new double[] {0.0, 0.0}); //TODO Tune this
        coralCoordinates.put(Positions.L3, new double[] {0.0, 0.0}); //TODO Tune this
        coralCoordinates.put(Positions.L4, new double[] {0.0, 0.0}); //TODO Tune this
        coralCoordinates.put(Positions.NET, new double[] {0.0, 0.0}); //TODO Tune this
        coralCoordinates.put(Positions.HP, new double[] {0.0, 0.0}); //TODO Tune this

        coralCoordinates.put(Positions.STOW, new double[] {0.0, 0.0}); //TODO Tune this

        return coralCoordinates;
    }

    private static HashMap<Positions, double[]> createAlgaeCoordinateMap() {
        HashMap<Positions, double[]> algaeCoordinates = new HashMap<>();

        algaeCoordinates.put(Positions.L1, new double[] {0.0, 0.0}); //TODO Tune this
        algaeCoordinates.put(Positions.L2, new double[] {0.0, 0.0}); //TODO Tune this
        algaeCoordinates.put(Positions.L3, new double[] {0.0, 0.0}); //TODO Tune this
        algaeCoordinates.put(Positions.L4, new double[] {0.0, 0.0}); //TODO Tune this
        algaeCoordinates.put(Positions.NET, new double[] {0.0, 0.0}); //TODO Tune this
        algaeCoordinates.put(Positions.HP, new double[] {0.0, 0.0}); //TODO Tune this
        algaeCoordinates.put(Positions.INTAKE, new double[] {0.0, 0.0}); //TODO Tune this
        algaeCoordinates.put(Positions.STOW, new double[] {0.0, 0.0}); //TODO Tune this

        return algaeCoordinates;
    }

    public static double[] getCoordinate(Positions requested) {

        if (getCurrentState() == RobotStateMachine.Algae) {
            currentCoordinateBase = algae;
        } else {
            currentCoordinateBase = coral;
        }

        double[] coordinates = currentCoordinateBase.get(requested);
        if (coordinates == null) {
            throw new IllegalArgumentException(
                    "No coordinates defined for " + requested + " in " + getCurrentState() + " mode");
        }

        return coordinates;
    }

    public static RobotStateMachine getCurrentState() {
        return currentState;
    }

    public void setRobotState(RobotStateMachine newState) {
        currentState = newState;
    }

    public void toggleRobotState() {
        if (currentState == RobotStateMachine.Algae) {
            currentState = RobotStateMachine.Coral;
        } else {
            currentState = RobotStateMachine.Algae;
        }
    }
}