package frc.robot.util;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FlippingUtil;
import frc.robot.FieldLayout;

/**
 * Utility for computing alliance-aware reef alignment poses.
 */
public final class AutoAlignHelper {
    private AutoAlignHelper() {}

    private static final double STANDOFF_METERS = Units.inchesToMeters(21.5);
    private static final double LATERAL_OFFSET_METERS = Units.inchesToMeters(6.5);
    private static final int[] FACE_TAG_IDS = {18, 17, 22, 21, 20, 19};

    private enum ScoringPosition {
        A(0, 1),
        B(0, -1),
        C(1, 1),
        D(1, -1),
        E(2, 1),
        F(2, -1),
        G(3, 1),
        H(3, -1),
        I(4, 1),
        J(4, -1),
        K(5, 1),
        L(5, -1);

        final int faceIndex;
        final int lateralSign;

        ScoringPosition(int faceIndex, int lateralSign) {
            this.faceIndex = faceIndex;
            this.lateralSign = lateralSign;
        }
    }

    private static final Map<Character, ScoringPosition> BLUE_BRANCH_MAPPING = Map.ofEntries(
        Map.entry('A', ScoringPosition.A),
        Map.entry('B', ScoringPosition.B),
        Map.entry('C', ScoringPosition.C),
        Map.entry('D', ScoringPosition.D),
        Map.entry('E', ScoringPosition.E),
        Map.entry('F', ScoringPosition.F),
        Map.entry('G', ScoringPosition.G),
        Map.entry('H', ScoringPosition.H),
        Map.entry('I', ScoringPosition.I),
        Map.entry('J', ScoringPosition.J),
        Map.entry('K', ScoringPosition.K),
        Map.entry('L', ScoringPosition.L)
    );

    private static final Map<Character, ScoringPosition> RED_BRANCH_MAPPING = Map.ofEntries(
        Map.entry('G', ScoringPosition.A),
        Map.entry('H', ScoringPosition.B),
        Map.entry('I', ScoringPosition.C),
        Map.entry('J', ScoringPosition.D),
        Map.entry('K', ScoringPosition.E),
        Map.entry('L', ScoringPosition.F),
        Map.entry('A', ScoringPosition.G),
        Map.entry('B', ScoringPosition.H),
        Map.entry('C', ScoringPosition.I),
        Map.entry('D', ScoringPosition.J),
        Map.entry('E', ScoringPosition.K),
        Map.entry('F', ScoringPosition.L)
    );

    /**
     * Computes the robot pose the drivetrain should drive to in order to align with
     * the requested branch on the reef.
     *
     * @param branch The desired reef branch, case-insensitive.
     * @return Field-relative pose for the robot to reach.
     */
    public static Pose2d computeBranchPose(char branch) {
        char normalized = Character.toUpperCase(branch);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        ScoringPosition position = (alliance == Alliance.Red ? RED_BRANCH_MAPPING : BLUE_BRANCH_MAPPING).get(normalized);
        if (position == null) {
            throw new IllegalArgumentException("Unknown reef branch: " + branch);
        }

        Pose2d facePose = getFacePose(position.faceIndex);

        Translation2d normal = new Translation2d(facePose.getRotation().getCos(), facePose.getRotation().getSin());
        Translation2d tangent = new Translation2d(-normal.getY(), normal.getX());

        Translation2d targetTranslation = facePose.getTranslation()
            .plus(normal.times(STANDOFF_METERS))
            .plus(tangent.times(position.lateralSign * LATERAL_OFFSET_METERS));

        Pose2d targetPose = new Pose2d(targetTranslation, facePose.getRotation());
        if (alliance == Alliance.Red) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
        }

        return targetPose;
    }

    private static Pose2d getFacePose(int faceIndex) {
        int tagId = FACE_TAG_IDS[Math.floorMod(faceIndex, FACE_TAG_IDS.length)];
        for (AprilTag tag : FieldLayout.AprilTags.APRIL_TAG_POSE) {
            if (tag.ID == tagId) {
                return tag.pose.toPose2d();
            }
        }
        throw new IllegalStateException("AprilTag ID " + tagId + " not defined in FieldLayout");
    }
}
