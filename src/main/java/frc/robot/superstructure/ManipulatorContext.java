package frc.robot.superstructure;

import frc.robot.SubsystemManager;
import frc.robot.subsystems.DifferentialWrist;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Pivot;

/**
 * Context passed to plan steps for callbacks and predicates.
 */
public final class ManipulatorContext {
    private final SubsystemManager manager;
    private final Pivot pivot;
    private final Elevator elevator;
    private final DifferentialWrist wrist;
    private final GroundIntake groundIntake;
    private final EndEffector endEffector;
    private final GamePieceState pieceState;
    private final PieceSensors sensors;

    public ManipulatorContext(
            SubsystemManager manager,
            Pivot pivot,
            Elevator elevator,
            DifferentialWrist wrist,
            GroundIntake groundIntake,
            EndEffector endEffector,
            GamePieceState pieceState,
            PieceSensors sensors
    ) {
        this.manager = manager;
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
        this.groundIntake = groundIntake;
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

    public DifferentialWrist wrist() {
        return wrist;
    }

    public GroundIntake groundIntake() {
        return groundIntake;
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
