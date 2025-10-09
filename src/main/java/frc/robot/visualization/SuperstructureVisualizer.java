package frc.robot.visualization;

import java.util.Objects;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/**
 * Builds the pose tree used by AdvantageScope's 2D/3D mechanism views.
 * <p>
 * Mechanism layout:
 * Pivot joint → three-stage elevator → differential wrist (pitch + roll) → end effector.
 * Everything is computed robot-relative so the geometry can be visualised in both simulation and
 * AdvantageKit log replays.
 * </p>
 */
public final class SuperstructureVisualizer {
    public record MechanismDimensions(
            Translation3d pivotMountTranslation,
            Translation3d pivotToElevatorOffset,
            double[] elevatorStageRestLengths,
            double[] elevatorStageMaxExtensions,
            Translation3d elevatorToPitchOffset,
            Translation3d pitchToRollOffset,
            Translation3d rollToEndEffectorOffset
    ) {
        private static final int EXPECTED_STAGE_COUNT = 3;

        public MechanismDimensions {
            pivotMountTranslation = Objects.requireNonNull(pivotMountTranslation);
            pivotToElevatorOffset = Objects.requireNonNull(pivotToElevatorOffset);
            elevatorStageRestLengths = Objects.requireNonNull(elevatorStageRestLengths).clone();
            elevatorStageMaxExtensions = Objects.requireNonNull(elevatorStageMaxExtensions).clone();
            elevatorToPitchOffset = Objects.requireNonNull(elevatorToPitchOffset);
            pitchToRollOffset = Objects.requireNonNull(pitchToRollOffset);
            rollToEndEffectorOffset = Objects.requireNonNull(rollToEndEffectorOffset);

            if (elevatorStageRestLengths.length != EXPECTED_STAGE_COUNT
                    || elevatorStageMaxExtensions.length != EXPECTED_STAGE_COUNT) {
                throw new IllegalArgumentException(
                        "MechanismDimensions expects exactly three elevator stages.");
            }
        }

        @Override
        public double[] elevatorStageRestLengths() {
            return elevatorStageRestLengths.clone();
        }

        @Override
        public double[] elevatorStageMaxExtensions() {
            return elevatorStageMaxExtensions.clone();
        }
    }

    private record MechanismSample(
            Pose3d pivotPose,
            Translation3d[] elevatorStagePoints,
            Pose3d pitchPose,
            Pose3d rollPose,
            Pose3d endEffectorPose
    ) {
        public MechanismSample {
            pivotPose = Objects.requireNonNull(pivotPose);
            elevatorStagePoints = Objects.requireNonNull(elevatorStagePoints).clone();
            pitchPose = Objects.requireNonNull(pitchPose);
            rollPose = Objects.requireNonNull(rollPose);
            endEffectorPose = Objects.requireNonNull(endEffectorPose);
        }

        @Override
        public Translation3d[] elevatorStagePoints() {
            return elevatorStagePoints.clone();
        }
    }

    private static MechanismDimensions defaultDimensions() {
        return new MechanismDimensions(
                Constants.SuperstructureVisualization.PIVOT_MOUNT,
                Constants.SuperstructureVisualization.PIVOT_TO_ELEVATOR,
                Constants.SuperstructureVisualization.ELEVATOR_STAGE_REST_LENGTHS,
                Constants.SuperstructureVisualization.ELEVATOR_STAGE_MAX_EXTENSIONS,
                Constants.SuperstructureVisualization.ELEVATOR_TO_PITCH,
                Constants.SuperstructureVisualization.PITCH_TO_ROLL,
                Constants.SuperstructureVisualization.ROLL_TO_END_EFFECTOR
        );
    }

    private static final double MECHANISM_VIEW_WIDTH_METERS = 3.0;
    private static final double MECHANISM_VIEW_HEIGHT_METERS = 3.0;
    private static final double MECHANISM_ROOT_X_OFFSET = MECHANISM_VIEW_WIDTH_METERS / 2.0;
    private static final double MECHANISM_ROOT_Y_OFFSET = 0.25;
    private static final double ANGLE_EPSILON = 1e-6;

    private static final Color8Bit[] ELEVATOR_STAGE_COLORS = {
            new Color8Bit(255, 215, 0),
            new Color8Bit(30, 144, 255),
            new Color8Bit(50, 205, 50)
    };
    private static final Color8Bit PITCH_LINK_COLOR = new Color8Bit(186, 85, 211);
    private static final Color8Bit END_EFFECTOR_COLOR = new Color8Bit(255, 105, 180);

    private static final String OFFSETS_TAB_NAME = "Superstructure Viz";
    private static final String PIVOT_OFFSET_TITLE = "Pivot Offset (deg)";
    private static final String ELEVATOR_OFFSET_TITLE = "Elevator Offset (m)";
    private static final String PITCH_OFFSET_TITLE = "Pitch Offset (deg)";
    private static final String ROLL_OFFSET_TITLE = "Roll Offset (deg)";

    private final Pivot pivot;
    private final Elevator elevator;
    private final DifferentialWrist wrist;
    private final MechanismDimensions dimensions;

    private final int elevatorStageCount;
    private final MechanismSample zeroedSample;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d pivotRoot;
    private final LoggedMechanismLigament2d pivotToElevatorLigament;
    private final LoggedMechanismLigament2d[] elevatorStageLigaments;
    private final LoggedMechanismLigament2d pitchLigament;
    private final LoggedMechanismLigament2d endEffectorLigament;

    private double pivotToElevatorAbsAngleRadians = 0.0;
    private final double[] elevatorStageAbsAnglesRadians;
    private double pitchAbsAngleRadians = 0.0;
    private double endEffectorAbsAngleRadians = 0.0;

    private final boolean offsetsEnabled;
    private GenericEntry pivotOffsetDegEntry;
    private GenericEntry elevatorOffsetMetersEntry;
    private GenericEntry pitchOffsetDegEntry;
    private GenericEntry rollOffsetDegEntry;

    public SuperstructureVisualizer(Pivot pivot, Elevator elevator, DifferentialWrist wrist) {
        this(pivot, elevator, wrist, defaultDimensions());
    }

    public SuperstructureVisualizer(
            Pivot pivot,
            Elevator elevator,
            DifferentialWrist wrist,
            MechanismDimensions dimensions
    ) {
        this.pivot = Objects.requireNonNull(pivot);
        this.elevator = Objects.requireNonNull(elevator);
        this.wrist = Objects.requireNonNull(wrist);
        this.dimensions = Objects.requireNonNull(dimensions);
        this.elevatorStageCount = this.dimensions.elevatorStageRestLengths().length;
        this.elevatorStageAbsAnglesRadians = new double[elevatorStageCount];

        this.zeroedSample = computeMechanismSample(0.0, 0.0, 0.0, 0.0);

        mechanism = new LoggedMechanism2d(MECHANISM_VIEW_WIDTH_METERS, MECHANISM_VIEW_HEIGHT_METERS);
        Translation3d zeroedPivotTranslation = zeroedSample.pivotPose().getTranslation();
        pivotRoot = mechanism.getRoot(
                "Pivot",
                MECHANISM_ROOT_X_OFFSET + zeroedPivotTranslation.getX(),
                MECHANISM_ROOT_Y_OFFSET + zeroedPivotTranslation.getZ()
        );

        pivotToElevatorLigament = pivotRoot.append(
                new LoggedMechanismLigament2d("PivotToElevator", 0.0, 0.0, 6.0, new Color8Bit(255, 165, 0))
        );

        elevatorStageLigaments = new LoggedMechanismLigament2d[elevatorStageCount];
        LoggedMechanismLigament2d appendPoint = pivotToElevatorLigament;
        for (int i = 0; i < elevatorStageCount; i++) {
            Color8Bit color = ELEVATOR_STAGE_COLORS[i % ELEVATOR_STAGE_COLORS.length];
            elevatorStageLigaments[i] = appendPoint.append(
                    new LoggedMechanismLigament2d("ElevatorStage" + (i + 1), 0.0, 0.0, 6.0, color)
            );
            appendPoint = elevatorStageLigaments[i];
        }

        pitchLigament = appendPoint.append(
                new LoggedMechanismLigament2d("PitchLink", 0.0, 0.0, 6.0, PITCH_LINK_COLOR)
        );
        endEffectorLigament = pitchLigament.append(
                new LoggedMechanismLigament2d("EndEffector", 0.0, 0.0, 6.0, END_EFFECTOR_COLOR)
        );

        offsetsEnabled = RobotBase.isSimulation();
        if (offsetsEnabled) {
            setupShuffleboardOffsets();
        }

        refreshMechanism2d(zeroedSample);
    }

    public void update() {
        double pivotPosition = pivot.getPivotPosition();
        double elevatorPosition = elevator.getElevatorPosition();
        double pitchPosition = wrist.getPitchPosition();
        double rollPosition = wrist.getRollPosition();

        MechanismSample measuredSample = computeMechanismSample(
                pivotPosition,
                elevatorPosition,
                pitchPosition,
                rollPosition
        );

        MechanismSample visualSample = offsetsEnabled
                ? computeMechanismSample(
                        pivotPosition + getPivotOffsetRadians(),
                        Math.max(0.0, elevatorPosition + getElevatorOffsetMeters()),
                        pitchPosition + getPitchOffsetRadians(),
                        rollPosition + getRollOffsetRadians()
                )
                : measuredSample;

        refreshMechanism2d(visualSample);
        Logger.recordOutput("Mechanism2d/Superstructure", mechanism);

        Logger.recordOutput("ZereodComponentPoses", packComponentPoses(zeroedSample));
        Logger.recordOutput("FinalComponentPoses", packComponentPoses(measuredSample));
        logSampleData(measuredSample);

        if (offsetsEnabled) {
            Logger.recordOutput(
                    "Superstructure/VisualizationOffsets",
                    new double[] {
                            Math.toDegrees(getPivotOffsetRadians()),
                            getElevatorOffsetMeters(),
                            Math.toDegrees(getPitchOffsetRadians()),
                            Math.toDegrees(getRollOffsetRadians())
                    }
            );
        }
    }

    private MechanismSample computeMechanismSample(
            double pivotRadians,
            double elevatorMeters,
            double pitchRadians,
            double rollRadians
    ) {
        double pivotAngle = pivotRadians
                - Constants.SuperstructureVisualization.PIVOT_STOW_SENSOR_RADIANS
                + Constants.SuperstructureVisualization.PIVOT_STOW_VISUAL_RADIANS;
        Rotation3d pivotRotation = new Rotation3d(0.0, pivotAngle, 0.0);
        Translation3d pivotTranslation = dimensions.pivotMountTranslation();
        Pose3d pivotPose = new Pose3d(pivotTranslation, pivotRotation);

        Translation3d elevatorAxisUnit = new Translation3d(0.0, 0.0, 1.0).rotateBy(pivotRotation);

        double[] stageRestLengths = dimensions.elevatorStageRestLengths().clone();
        double[] stageMaxExtensions = dimensions.elevatorStageMaxExtensions();
        double maxTotalExtension = 0.0;
        for (double stage : stageMaxExtensions) {
            maxTotalExtension += Math.max(stage, 0.0);
        }
        double clampedExtension = Math.min(Math.max(elevatorMeters, 0.0), maxTotalExtension);

        double[] stageExtensions = new double[elevatorStageCount];
        double firstStageFixedExtension = elevatorStageCount > 0
                ? Math.max(stageMaxExtensions[0], 0.0)
                : 0.0;

        if (elevatorStageCount > 0) {
            stageRestLengths[0] += firstStageFixedExtension;
            stageExtensions[0] = 0.0;
        }

        double remainingExtension = Math.max(0.0, clampedExtension - firstStageFixedExtension);
        for (int i = 0; i < elevatorStageCount; i++) {
            double stageCapacity = i == 0 ? 0.0 : Math.max(stageMaxExtensions[i], 0.0);
            double extension = Math.min(remainingExtension, stageCapacity);
            stageExtensions[i] = extension;
            remainingExtension -= extension;
        }

        Translation3d[] stagePoints = new Translation3d[elevatorStageCount + 1];
        stagePoints[0] = pivotTranslation.plus(dimensions.pivotToElevatorOffset().rotateBy(pivotRotation));

        Translation3d currentPoint = stagePoints[0];
        for (int i = 0; i < elevatorStageCount; i++) {
            double segmentLength = stageRestLengths[i] + stageExtensions[i];
            Translation3d segmentVector = elevatorAxisUnit.times(segmentLength);
            currentPoint = currentPoint.plus(segmentVector);
            stagePoints[i + 1] = currentPoint;
        }

        Translation3d pitchTranslation = stagePoints[stagePoints.length - 1]
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

        return new MechanismSample(pivotPose, stagePoints, pitchPose, rollPose, endEffectorPose);
    }

    private void refreshMechanism2d(MechanismSample sample) {
        Translation2d pivotPoint = projectTo2d(sample.pivotPose().getTranslation());
        pivotRoot.setPosition(
                MECHANISM_ROOT_X_OFFSET + pivotPoint.getX(),
                MECHANISM_ROOT_Y_OFFSET + pivotPoint.getY()
        );

        Translation3d[] stagePoints3d = sample.elevatorStagePoints();

        Translation2d stageBasePoint = projectTo2d(stagePoints3d[0]);
        pivotToElevatorAbsAngleRadians = updateLigament(
                pivotToElevatorLigament,
                pivotPoint,
                stageBasePoint,
                pivotToElevatorAbsAngleRadians,
                0.0
        );

        double parentAngle = pivotToElevatorAbsAngleRadians;
        for (int i = 0; i < elevatorStageLigaments.length; i++) {
            Translation2d startPoint = projectTo2d(stagePoints3d[i]);
            Translation2d endPoint = projectTo2d(stagePoints3d[i + 1]);
            parentAngle = elevatorStageAbsAnglesRadians[i] = updateLigament(
                    elevatorStageLigaments[i],
                    startPoint,
                    endPoint,
                    elevatorStageAbsAnglesRadians[i],
                    parentAngle
            );
        }

        Translation2d stageTopPoint = projectTo2d(stagePoints3d[stagePoints3d.length - 1]);
        Translation2d pitchPoint = projectTo2d(sample.pitchPose().getTranslation());
        pitchAbsAngleRadians = updateLigament(
                pitchLigament,
                stageTopPoint,
                pitchPoint,
                pitchAbsAngleRadians,
                parentAngle
        );

        Translation2d endEffectorPoint = projectTo2d(sample.endEffectorPose().getTranslation());
        endEffectorAbsAngleRadians = updateLigament(
                endEffectorLigament,
                pitchPoint,
                endEffectorPoint,
                endEffectorAbsAngleRadians,
                pitchAbsAngleRadians
        );
    }

    private static Pose3d[] packComponentPoses(MechanismSample sample) {
        Translation3d[] elevatorStagePoints = sample.elevatorStagePoints();
        Rotation3d stageOrientation = sample.pivotPose().getRotation();

        Pose3d[] poses = new Pose3d[2 + elevatorStagePoints.length + 2];
        poses[0] = sample.endEffectorPose();
        poses[1] = sample.pivotPose();
        for (int i = 0; i < elevatorStagePoints.length; i++) {
            poses[2 + i] = new Pose3d(elevatorStagePoints[i], stageOrientation);
        }
        poses[2 + elevatorStagePoints.length] = sample.pitchPose();
        poses[3 + elevatorStagePoints.length] = sample.rollPose();
        return poses;
    }

    private static double[][] toArray(Translation3d[] points) {
        double[][] array = new double[points.length][3];
        for (int i = 0; i < points.length; i++) {
            array[i][0] = points[i].getX();
            array[i][1] = points[i].getY();
            array[i][2] = points[i].getZ();
        }
        return array;
    }

    private static double[] computeStageLengths(Translation3d[] points) {
        if (points.length <= 1) {
            return new double[0];
        }
        double[] lengths = new double[points.length - 1];
        for (int i = 0; i < lengths.length; i++) {
            lengths[i] = points[i + 1].minus(points[i]).getNorm();
        }
        return lengths;
    }

    private void logSampleData(MechanismSample sample) {
        Translation3d[] stagePoints = sample.elevatorStagePoints();
        Translation2d endEffectorPoint = projectTo2d(sample.endEffectorPose().getTranslation());

        Logger.recordOutput(
                "Superstructure/EndEffector2d",
                new double[] {endEffectorPoint.getX(), endEffectorPoint.getY()}
        );
        Logger.recordOutput("Superstructure/ElevatorStagePoints", toArray(stagePoints));
        Logger.recordOutput("Superstructure/ElevatorStageLengths", computeStageLengths(stagePoints));
    }

    private void setupShuffleboardOffsets() {
        ShuffleboardTab tab = Shuffleboard.getTab(OFFSETS_TAB_NAME);
        Map<String, Object> angleProps = Map.of("min", -180.0, "max", 180.0);
        Map<String, Object> elevatorProps = Map.of("min", -0.3, "max", 0.3);

        pivotOffsetDegEntry = getOrCreateSlider(tab, PIVOT_OFFSET_TITLE, 0.0, angleProps);
        elevatorOffsetMetersEntry = getOrCreateSlider(tab, ELEVATOR_OFFSET_TITLE, 0.0, elevatorProps);
        pitchOffsetDegEntry = getOrCreateSlider(tab, PITCH_OFFSET_TITLE, 0.0, angleProps);
        rollOffsetDegEntry = getOrCreateSlider(tab, ROLL_OFFSET_TITLE, 0.0, angleProps);
    }

    private GenericEntry getOrCreateSlider(
            ShuffleboardTab tab,
            String title,
            double defaultValue,
            Map<String, Object> properties
    ) {
        for (ShuffleboardComponent<?> component : tab.getComponents()) {
            if (component.getTitle().equals(title) && component instanceof SimpleWidget widget) {
                widget.withWidget(BuiltInWidgets.kNumberSlider).withProperties(properties);
                return widget.getEntry();
            }
        }
        return tab.add(title, defaultValue)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(properties)
                .getEntry();
    }

    private double getPivotOffsetRadians() {
        return Math.toRadians(getEntryValue(pivotOffsetDegEntry));
    }

    private double getElevatorOffsetMeters() {
        return getEntryValue(elevatorOffsetMetersEntry);
    }

    private double getPitchOffsetRadians() {
        return Math.toRadians(getEntryValue(pitchOffsetDegEntry));
    }

    private double getRollOffsetRadians() {
        return Math.toRadians(getEntryValue(rollOffsetDegEntry));
    }

    private static double getEntryValue(GenericEntry entry) {
        return entry != null ? entry.getDouble(0.0) : 0.0;
    }

    private double updateLigament(
            LoggedMechanismLigament2d ligament,
            Translation2d start,
            Translation2d end,
            double lastAbsoluteAngleRadians,
            double parentAbsoluteAngleRadians
    ) {
        Translation2d vector = end.minus(start);
        double length = vector.getNorm();

        ligament.setLength(length);

        if (length <= ANGLE_EPSILON) {
            ligament.setAngle(0.0);
            return parentAbsoluteAngleRadians;
        }

        double absoluteAngle = Math.atan2(vector.getY(), vector.getX());
        double relativeAngleDegrees = Math.toDegrees(absoluteAngle - parentAbsoluteAngleRadians);
        ligament.setAngle(relativeAngleDegrees);

        return absoluteAngle;
    }

    private static Translation2d projectTo2d(Translation3d translation) {
        return new Translation2d(translation.getX(), translation.getZ());
    }
}
