package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final VisionIO io;
    private final VisionIOInputs inputs = new VisionIOInputs();
    private final List<VisionObservation> results = new ArrayList<>();

    public VisionSubsystem(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        results.clear();
        results.addAll(inputs.observations);
    }

    public List<VisionObservation> getResults() {
        return List.copyOf(results);
    }

    public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        io.setRobotPoseSupplier(supplier);
    }
}