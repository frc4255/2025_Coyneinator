package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.GroundIntake;

/**
 * Minimal AdvantageScope visualizer for the ground intake. Produces poses for the base mount and
 * roller so existing visual layouts continue to work with the reduced robot.
 */
public final class GroundIntakeVisualizer {
    private final GroundIntake groundIntake;
    private final NetworkTableEntry demoPitchRadiansEntry;

    public GroundIntakeVisualizer(GroundIntake groundIntake) {
        this.groundIntake = groundIntake;

        NetworkTable table = NetworkTableInstance.getDefault()
            .getTable("GroundIntakeVisualizer")
            .getSubTable("ManualPose");
        demoPitchRadiansEntry = table.getEntry("PitchRadians");
        demoPitchRadiansEntry.setDefaultDouble(0.0);
    }

    public void update() {
        double measuredPitch = groundIntake.getPitchPosition();
        double goalPitch = groundIntake.getPitchGoalPosition();
        double demoPitch = demoPitchRadiansEntry.getDouble(0.0);

        Logger.recordOutput("GroundIntakeVisualizer/MeasuredRadians", measuredPitch);
        Logger.recordOutput("GroundIntakeVisualizer/GoalRadians", goalPitch);

        Pose3d basePose = new Pose3d(Constants.GroundIntake.VISUALIZER_BASE_MOUNT, new Rotation3d());
        Pose3d measuredPose = toRollerPose(measuredPitch);
        Pose3d goalPose = toRollerPose(goalPitch);
        Pose3d demoPose = toRollerPose(demoPitch);

        Logger.recordOutput("GroundIntakeVisualizer/BasePose", basePose);
        Logger.recordOutput("GroundIntakeVisualizer/MeasuredPose", measuredPose);
        Logger.recordOutput("GroundIntakeVisualizer/GoalPose", goalPose);
        Logger.recordOutput("GroundIntakeVisualizer/DemoPose", demoPose);
    }

    private Pose3d toRollerPose(double pitchRadians) {
        Rotation3d rotation = new Rotation3d(0.0, -pitchRadians, 0.0);
        Transform3d transform = new Transform3d(Constants.GroundIntake.VISUALIZER_ROLLER_OFFSET, rotation);
        return new Pose3d(Constants.GroundIntake.VISUALIZER_BASE_MOUNT, new Rotation3d()).transformBy(transform);
    }
}
