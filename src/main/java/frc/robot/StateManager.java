package frc.robot;

public class StateManager {
    public enum RobotStateMachine {
        Coral,
        Algae
    }

    private RobotStateMachine currentState = RobotStateMachine.Coral;

    public StateManager() {
    }

    public RobotStateMachine getCurrentState() {
        return currentState;
    }

    public void setRobotState(RobotStateMachine newState) {
        currentState = newState;
    }
}