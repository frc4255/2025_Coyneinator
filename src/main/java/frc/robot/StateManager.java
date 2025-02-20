package frc.robot;

import java.lang.Thread.State;
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

    
    private static final HashMap<Positions, double[]> coral = new HashMap<>();
    private static final HashMap<Positions, double[]> algae = new HashMap<>();

    private static HashMap<Positions, double[]> currentCoordinateBase = new HashMap<>();

    public StateManager() {

        //double[] = {Elevator Extension, ElevatorPitchANGLERAD, WristPitchAngleRAD, WristRollAngleRAD}

        coral.put(Positions.L1, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        coral.put(Positions.L2, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        coral.put(Positions.L3, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        coral.put(Positions.L4, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        coral.put(Positions.NET, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        coral.put(Positions.HP, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        coral.put(Positions.STOW, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this

        algae.put(Positions.L1, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        algae.put(Positions.L2, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        algae.put(Positions.L3, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        algae.put(Positions.L4, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        algae.put(Positions.NET, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        algae.put(Positions.HP, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        algae.put(Positions.INTAKE, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this
        algae.put(Positions.STOW, new double[] {0.0, 0.0, 0.0, 0.0}); //TODO Tune this

    }

    public static double[] getCoordinate(Positions requested) {

        if (getCurrentState() == RobotStateMachine.Algae) {
            currentCoordinateBase = algae;
        } else {
            currentCoordinateBase = coral;
        }

        return currentCoordinateBase.get(requested);
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