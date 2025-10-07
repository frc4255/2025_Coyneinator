package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

    private final VisionIO io;
    private final VisionIOInputs inputs = new VisionIOInputs();
    private final List<VisionObservation> results = new ArrayList<>();
    private final List<String> knownCameraNames = new ArrayList<>();
    private static final int DASHBOARD_UPDATE_INTERVAL = 3;
    private int dashboardCounter = 0;

    public VisionSubsystem(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        results.clear();
        results.addAll(inputs.observations);

        Logger.recordOutput("Vision/ObservationCount", results.size());
        dashboardCounter = (dashboardCounter + 1) % DASHBOARD_UPDATE_INTERVAL;
        if (dashboardCounter == 0) {
            for (String cameraName : knownCameraNames) {
                String baseKey = "Vision/Cameras/" + cameraName;
                Logger.recordOutput(baseKey + "/HasObservation", false);
                SmartDashboard.putBoolean(baseKey + "/HasObservation", false);
                SmartDashboard.putNumberArray(baseKey + "/Pose", new double[] {});
                SmartDashboard.putNumber(baseKey + "/Timestamp", 0.0);
                SmartDashboard.putNumber(baseKey + "/StdDev", 0.0);
            }

            for (VisionObservation observation : results) {
                String baseKey = "Vision/Cameras/" + observation.cameraName();
                Logger.recordOutput(baseKey + "/HasObservation", true);
                Logger.recordOutput(baseKey + "/Pose", Pose2d.struct, observation.pose());
                Logger.recordOutput(baseKey + "/Timestamp", observation.timestamp());
                Logger.recordOutput(baseKey + "/StdDev", observation.stdDev());

                SmartDashboard.putBoolean(baseKey + "/HasObservation", true);
                SmartDashboard.putNumberArray(
                    baseKey + "/Pose",
                    new double[] {
                        observation.pose().getX(),
                        observation.pose().getY(),
                        observation.pose().getRotation().getDegrees()
                    }
                );
                SmartDashboard.putNumber(baseKey + "/Timestamp", observation.timestamp());
                SmartDashboard.putNumber(baseKey + "/StdDev", observation.stdDev());
            }
        }
    }

    public List<VisionObservation> getResults() {
        return List.copyOf(results);
    }

    public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        io.setRobotPoseSupplier(supplier);
    }

    public void setKnownCameraNames(List<String> cameraNames) {
        knownCameraNames.clear();
        if (cameraNames != null) {
            knownCameraNames.addAll(cameraNames);
        }
    }
}
