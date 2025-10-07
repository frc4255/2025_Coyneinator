package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldLayout;
import frc.robot.StateManager;

/** Utility presets for seeding {@link AutoAlignConfig} data when desired. */
public final class AutoAlignPresets {
    private AutoAlignPresets() {}

    public static AutoAlignConfig orientation() {
        AutoAlignConfig.Builder builder = AutoAlignConfig.builder();

        builder
            .addAllianceTag(1, getTagPose(18), getTagPose(7))
            .addAllianceTag(2, getTagPose(17), getTagPose(8))
            .addAllianceTag(3, getTagPose(22), getTagPose(9))
            .addAllianceTag(4, getTagPose(21), getTagPose(10))
            .addAllianceTag(5, getTagPose(20), getTagPose(11))
            .addAllianceTag(6, getTagPose(19), getTagPose(6));

        addLevelButtons(builder);
        addScoringButtons(builder);

        return builder.build();
    }

    private static Pose2d getTagPose(int tag) {
        return FieldLayout.AprilTags.APRIL_TAG_POSE.get(tag).pose.toPose2d();
    }

    private static void addLevelButtons(AutoAlignConfig.Builder builder) {
        builder
            .addLevelButton(13, StateManager.Positions.L1)
            .addLevelButton(14, StateManager.Positions.L2)
            .addLevelButton(15, StateManager.Positions.L3)
            .addLevelButton(16, StateManager.Positions.L4);
    }

    private static void addScoringButtons(AutoAlignConfig.Builder builder) {
        addPosition(builder, 1, 1, 90, -90, 0);
        addPosition(builder, 2, 1, -90, -90, 0);
        addPosition(builder, 3, 2, 150, 225, 60);
        addPosition(builder, 4, 2, -30, 225, 60);
        addPosition(builder, 5, 3, 150, 315, 120);
        addPosition(builder, 6, 3, -30, 315, 120);
        addPosition(builder, 7, 4, -90, 90, 180);
        addPosition(builder, 8, 4, 90, 90, 180);
        addPosition(builder, 9, 5, -30, 45, 240);
        addPosition(builder, 10, 5, 150, 45, 240);
        addPosition(builder, 11, 6, 30, 135, 300);
        addPosition(builder, 12, 6, 210, 135, 300);
    }

    private static void addPosition(
        AutoAlignConfig.Builder builder,
        int button,
        int tagSlot,
        double coralDirectionDeg,
        double robotDirectionDeg,
        double robotRotationDeg
    ) {
        builder.addScoringButton(
            button,
            tagSlot,
            polar(AutoAlignController.ROBOT_STAGING_RADIUS_METERS, robotDirectionDeg),
            polar(AutoAlignController.CORAL_OFFSET_METERS, coralDirectionDeg),
            Rotation2d.fromDegrees(robotRotationDeg)
        );
    }

    private static Translation2d polar(double magnitudeMeters, double directionDeg) {
        return new Translation2d(magnitudeMeters, Rotation2d.fromDegrees(directionDeg));
    }
}
