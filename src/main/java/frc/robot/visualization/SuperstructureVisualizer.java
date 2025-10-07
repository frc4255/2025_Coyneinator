package frc.robot.visualization;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/**
 * Builds the pose tree used by AdvantageScope's 3D mechanism view.
 * <p>
 * The manipulator stack is a pivot, followed by an elevator, then a differential wrist with pitch
 * and roll axes. Each pose is computed in robot-relative coordinates so AdvantageScope can render
 * the current configuration.
 * </p>
 */
public final class SuperstructureVisualizer {
    /**
     * Geometric description of the manipulator in its local coordinate frames.
     *
     * @param pivotMountTranslation Location of the pivot joint relative to the robot origin.
     * @param pivotToElevatorOffset Fixed offset from the pivot joint to the elevator carriage while
     *                              the elevator is fully retracted.
     * @param elevatorToPitchOffset Fixed offset from the top of the elevator to the wrist pitch
     *                              joint when the elevator is at zero extension.
     * @param pitchToRollOffset     Offset from the pitch joint to the roll joint in the pitch
     *                              coordinate frame.
     * @param rollToEndEffectorOffset Offset from the roll joint to the intake/end-effector origin in
     *                              the roll coordinate frame.
     */
    public record MechanismDimensions(
            Translation3d pivotMountTranslation,
            Translation3d pivotToElevatorOffset,
            Translation3d elevatorToPitchOffset,
            Translation3d pitchToRollOffset,
            Translation3d rollToEndEffectorOffset
    ) {}

    private static MechanismDimensions defaultDimensions() {
        return new MechanismDimensions(
                Constants.SuperstructureVisualization.PIVOT_MOUNT,
                Constants.SuperstructureVisualization.PIVOT_TO_ELEVATOR,
                Constants.SuperstructureVisualization.ELEVATOR_TO_PITCH,
                Constants.SuperstructureVisualization.PITCH_TO_ROLL,
                Constants.SuperstructureVisualization.ROLL_TO_END_EFFECTOR
        );
    }

    private final Pivot pivot;
    private final Elevator elevator;
    private final DifferentialWrist wrist;
    private final MechanismDimensions dimensions;
    private final Pose3d[] zeroedComponentPoses;

    public SuperstructureVisualizer(Pivot pivot, Elevator elevator, DifferentialWrist wrist) {
        this(pivot, elevator, wrist, defaultDimensions());
    }

    public SuperstructureVisualizer(
            Pivot pivot,
            Elevator elevator,
            DifferentialWrist wrist,
            MechanismDimensions dimensions
    ) {
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
        this.dimensions = dimensions;
        this.zeroedComponentPoses = computeComponentPoses(0.0, 0.0, 0.0, 0.0);
    }

    public void update() {
        Pose3d[] poses = computeComponentPoses(
                pivot.getPivotPosition(),
                elevator.getElevatorPosition(),
                wrist.getPitchPosition(),
                wrist.getRollPosition()
        );

        Logger.recordOutput("ZereodComponentPoses", zeroedComponentPoses);
        Logger.recordOutput("FinalComponentPoses", poses);
    }

    private Pose3d[] computeComponentPoses(
            double pivotRadians,
            double elevatorMeters,
            double pitchRadians,
            double rollRadians
    ) {
        Rotation3d pivotRotation = new Rotation3d(0.0, pivotRadians, 0.0);
        Translation3d pivotTranslation = dimensions.pivotMountTranslation();
        Pose3d pivotPose = new Pose3d(pivotTranslation, pivotRotation);

        Translation3d elevatorTranslation = pivotTranslation
                .plus(dimensions.pivotToElevatorOffset().rotateBy(pivotRotation))
                .plus(new Translation3d(elevatorMeters, 0.0, 0.0).rotateBy(pivotRotation));
        Pose3d elevatorPose = new Pose3d(elevatorTranslation, pivotRotation);

        Translation3d pitchTranslation = elevatorTranslation
                .plus(dimensions.elevatorToPitchOffset().rotateBy(pivotRotation));
        Rotation3d pitchRotation = pivotRotation.rotateBy(new Rotation3d(0.0, pitchRadians, 0.0));
        Pose3d pitchPose = new Pose3d(pitchTranslation, pitchRotation);

        Translation3d rollTranslation = pitchTranslation
                .plus(dimensions.pitchToRollOffset().rotateBy(pitchRotation));
        Rotation3d rollRotation = pitchRotation.rotateBy(new Rotation3d(rollRadians, 0.0, 0.0));
        Pose3d rollPose = new Pose3d(rollTranslation, rollRotation);

        Translation3d endEffectorTranslation = rollTranslation
                .plus(dimensions.rollToEndEffectorOffset().rotateBy(rollRotation));
        Pose3d endEffectorPose = new Pose3d(endEffectorTranslation, rollRotation);

        return new Pose3d[] {
                endEffectorPose,
                pivotPose,
                elevatorPose,
                pitchPose,
                rollPose
        };
    }
}
