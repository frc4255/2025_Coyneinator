package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionIOPhoton implements VisionIO {
    private final List<Camera> cameras;

    public VisionIOPhoton(List<CameraConfig> configs) {
        this.cameras = configs.stream()
            .map(config -> new Camera(new PhotonCamera(config.name()), config.robotToCamera()))
            .collect(Collectors.toList());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.observations.clear();
        for (Camera camera : cameras) {
            camera.updateEstimate();
            camera.updateCameraPoseEntry();
            camera.getEstimate().ifPresent(inputs.observations::add);
        }
    }

    @Override
    public void setRobotPoseSupplier(java.util.function.Supplier<Pose2d> supplier) {
        cameras.forEach(camera -> camera.setRobotPoseSupplier(supplier));
    }

    public static class CameraConfig {
        private final String name;
        private final Transform3d robotToCamera;

        public CameraConfig(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }

        public String name() {
            return name;
        }

        public Transform3d robotToCamera() {
            return robotToCamera;
        }
    }
}
