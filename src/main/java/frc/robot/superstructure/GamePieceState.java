package frc.robot.superstructure;

/**
 * Mutable record of the robot's game-piece possession state.
 */
public final class GamePieceState {
    private boolean coralInIntake;
    private boolean coralInWrist;
    private boolean algaeInIntake;
    private boolean algaeInEndEffector;

    public void updateFromSensors(PieceSensors sensors) {
        if (sensors.hasCoralIntakeSensor()) {
            coralInIntake = sensors.coralDetectedInIntake();
        }
        if (sensors.hasCoralWristSensor()) {
            coralInWrist = sensors.coralDetectedAtWrist();
        }
    }

    public boolean isCoralInIntake() {
        return coralInIntake;
    }

    public void setCoralInIntake(boolean value) {
        coralInIntake = value;
    }

    public boolean isCoralInWrist() {
        return coralInWrist;
    }

    public void setCoralInWrist(boolean value) {
        coralInWrist = value;
    }

    public boolean isAlgaeInIntake() {
        return algaeInIntake;
    }

    public void setAlgaeInIntake(boolean value) {
        algaeInIntake = value;
    }

    public boolean isAlgaeInEndEffector() {
        return algaeInEndEffector;
    }

    public void setAlgaeInEndEffector(boolean value) {
        algaeInEndEffector = value;
    }

    public void clearAll() {
        coralInIntake = false;
        coralInWrist = false;
        algaeInIntake = false;
        algaeInEndEffector = false;
    }
}
