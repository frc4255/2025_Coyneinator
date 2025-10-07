package frc.robot.subsystems.Vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    void updateInputs(VisionIOInputs inputs);

    default void setRobotPoseSupplier(Supplier<Pose2d> supplier) {}
}
