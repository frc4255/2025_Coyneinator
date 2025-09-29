package frc.robot.superstructure;

import frc.robot.SubsystemManager;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristRoll;

/**
 * Context passed to plan steps for callbacks and predicates.
 */
public final class ManipulatorContext {
    private final SubsystemManager manager;
    private final Pivot pivot;
    private final Elevator elevator;
    private final WristPitch wristPitch;
    private final WristRoll wristRoll;
    private final EndEffector endEffector;
    private final GamePieceState pieceState;
    private final PieceSensors sensors;

    public ManipulatorContext(
            SubsystemManager manager,
            Pivot pivot,
            Elevator elevator,
            WristPitch wristPitch,
            WristRoll wristRoll,
            EndEffector endEffector,
            GamePieceState pieceState,
            PieceSensors sensors
    ) {
        this.manager = manager;
        this.pivot = pivot;
        this.elevator = elevator;
        this.wristPitch = wristPitch;
        this.wristRoll = wristRoll;
        this.endEffector = endEffector;
        this.pieceState = pieceState;
        this.sensors = sensors;
    }

    public SubsystemManager manager() {
        return manager;
    }

    public Pivot pivot() {
        return pivot;
    }

    public Elevator elevator() {
        return elevator;
    }

    public WristPitch wristPitch() {
        return wristPitch;
    }

    public WristRoll wristRoll() {
        return wristRoll;
    }

    public EndEffector endEffector() {
        return endEffector;
    }

    public GamePieceState pieceState() {
        return pieceState;
    }

    public PieceSensors sensors() {
        return sensors;
    }
}
