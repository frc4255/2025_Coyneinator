package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Pivot;

/**
 * Builds the pose tree used by AdvantageScope's 3D mechanism view.
 * <p>
 * The manipulator stack is a pivot, followed by an elevator, then a differential wrist with pitch
 * and roll axes. Each pose is computed in robot-relative coordinates so AdvantageScope can render
 * the current configuration. A separate pose chain is also provided for the ground intake.
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
     */
    public record ManipulatorDimensions(
            Translation3d pivotMountTranslation,
            Translation3d pivotToElevatorOffset,
            Translation3d elevatorToPitchOffset,
            Translation3d pitchToRollOffset
    ) {}

    /**
     * Geometry for the ground intake mechanism.
     *
     * @param baseMountTranslation  Location of the ground intake pivot relative to the robot origin.
     * @param baseToRollerOffset    Offset from the pivot to the roller center when pitch is zero.
     */
    public record GroundIntakeDimensions(Translation3d baseMountTranslation) {}

    /**
     * Aggregated configuration for all visualized mechanisms so they can be tweaked in one place.
     */
    public static final class Config {
        public final ManipulatorDimensions manipulator;
        public final GroundIntakeDimensions groundIntake;

        public Config(ManipulatorDimensions manipulator, GroundIntakeDimensions groundIntake) {
            this.manipulator = manipulator;
            this.groundIntake = groundIntake;
        }

        public static Config fromConstants() {
            return new Config(
                    new ManipulatorDimensions(
                            Constants.SuperstructureVisualization.Manipulator.PIVOT_MOUNT,
                            Constants.SuperstructureVisualization.Manipulator.PIVOT_TO_ELEVATOR,
                            Constants.SuperstructureVisualization.Manipulator.ELEVATOR_TO_PITCH,
                            Constants.SuperstructureVisualization.Manipulator.PITCH_TO_ROLL
                    ),
                    new GroundIntakeDimensions(Constants.SuperstructureVisualization.GroundIntake.BASE_MOUNT)
            );
        }

        public Config withManipulatorDimensions(ManipulatorDimensions dimensions) {
            return new Config(dimensions, groundIntake);
        }

        public Config withGroundIntakeDimensions(GroundIntakeDimensions dimensions) {
            return new Config(manipulator, dimensions);
        }
    }

    private static Config defaultConfig() {
        return Config.fromConstants();
    }

    private final Pivot pivot;
    private final Elevator elevator;
    private final DifferentialWrist wrist;
    private final GroundIntake groundIntake;
    private final Config config;
    private final Pose3d[] zeroedManipulatorPoses;

    public SuperstructureVisualizer(
            Pivot pivot,
            Elevator elevator,
            DifferentialWrist wrist,
            GroundIntake groundIntake
    ) {
        this(pivot, elevator, wrist, groundIntake, defaultConfig());
    }

    public SuperstructureVisualizer(
            Pivot pivot,
            Elevator elevator,
            DifferentialWrist wrist,
            GroundIntake groundIntake,
            Config config
    ) {
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
        this.groundIntake = groundIntake;
        this.config = config;
        this.zeroedManipulatorPoses = new Pose3d[] {
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d(),
                new Pose3d()
        };
    }

    public Config getConfig() {
        return config;
    }

    public Pose3d[] getZeroedManipulatorPoses() {
        return zeroedManipulatorPoses.clone();
    }

    public void update() {
        double pivotRadians = pivot.getPivotPosition();
        double elevatorMeters = elevator.getElevatorPosition();
        double pitchRadians = wrist.getPitchPosition();
        double rollRadians = wrist.getRollPosition();
        double groundIntakePitch = groundIntake != null ? groundIntake.getPitchPosition() : 0.0;

        if (Constants.SuperstructureVisualization.ENABLE_ANIMATION) {
            double phase = 2.0 * Math.PI * Timer.getFPGATimestamp()
                    / Math.max(Constants.SuperstructureVisualization.ANIMATION_PERIOD_SECONDS, 0.01);
            double sinPhase = Math.sin(phase);

            pivotRadians += Math.toRadians(Constants.SuperstructureVisualization.ANIMATION_PIVOT_SWING_DEGREES) * sinPhase;

            double elevatorOffset = 0.5 * Constants.SuperstructureVisualization.ANIMATION_ELEVATOR_TRAVEL_METERS * sinPhase;
            elevatorMeters = Math.max(0.0, elevatorMeters + elevatorOffset);

            pitchRadians += Math.toRadians(Constants.SuperstructureVisualization.ANIMATION_PITCH_SWING_DEGREES) * sinPhase;

            // offset roll phase by 90 degrees to make motion distinct.
            double cosPhase = Math.cos(phase);
            rollRadians += Math.toRadians(Constants.SuperstructureVisualization.ANIMATION_ROLL_SWING_DEGREES) * cosPhase;

            groundIntakePitch += Math.toRadians(
                    Constants.SuperstructureVisualization.ANIMATION_GROUND_PITCH_SWING_DEGREES
            ) * sinPhase;
        }

        Pose3d[] poses = computeComponentPoses(
                pivotRadians,
                elevatorMeters,
                pitchRadians,
                rollRadians,
                groundIntakePitch
        );

        Logger.recordOutput("ZereodComponentPoses", zeroedManipulatorPoses);
        Logger.recordOutput("FinalComponentPoses", poses);
    }

    private Pose3d[] computeComponentPoses(
            double pivotRadians,
            double elevatorMeters,
            double pitchRadians,
            double rollRadians,
            double groundIntakePitchRadians
    ) {
        ManipulatorDimensions dimensions = config.manipulator;
        GroundIntakeDimensions groundDimensions = config.groundIntake;

        Rotation3d groundIntakeRotation = new Rotation3d(0.0, groundIntakePitchRadians, 0.0);
        Pose3d groundIntakePose = new Pose3d(groundDimensions.baseMountTranslation(), groundIntakeRotation);

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

        // Order matches AdvantageScope component stack: [groundIntake, pivot, elevator, pitch, roll].
        return new Pose3d[] {
                groundIntakePose,
                pivotPose,
                elevatorPose,
                pitchPose,
                rollPose
        };
    }
}
