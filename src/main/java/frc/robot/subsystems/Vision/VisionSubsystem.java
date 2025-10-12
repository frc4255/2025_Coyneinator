package frc.robot.subsystems.Vision;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    @AutoLog
    public static class VisionSubsystemInputs {
        public Pose2d[] estimatedPoses = new Pose2d[0];
        public double[] timestamps = new double[0];
        public double[] stdDeviations = new double[0];
    }
    
    /* Might need to create a custom class if I need more features. */
    private final Camera[] cameras;

    /* Possibly a list of poses generated from each individual camera */
    private final List<PoseAndTimestampAndDev> results = new ArrayList<>();

    private final VisionSubsystemInputsAutoLogged inputs = new VisionSubsystemInputsAutoLogged();

    public VisionSubsystem(Camera[] cameras) {
        this.cameras = Objects.requireNonNull(cameras, "camera array cannot be null").clone();
    }

    public byte[] convertToByteArray(Optional<PoseAndTimestampAndDev> optionalPose) {
        if (optionalPose.isPresent()) {
            PoseAndTimestampAndDev pose = optionalPose.get();
    
            try (ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
                 ObjectOutputStream objectOutputStream = new ObjectOutputStream(byteArrayOutputStream)) {
    
                objectOutputStream.writeObject(pose);
                return byteArrayOutputStream.toByteArray();
    
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return new byte[0]; // Return an empty byte array if the Optional is empty
    }
    
    @Override
    public void periodic() {

        results.clear();
        
        for (Camera cam : cameras) {
            if (cam == null) {
                continue;
            }
            cam.updateEstimate();
            cam.updateCameraPoseEntry();
            Optional<PoseAndTimestampAndDev> camEst = cam.getEstimate();
            camEst.ifPresent(pose -> {
                results.add(pose);
                Logger.recordOutput("Camera " + cam + " Pose", pose.getPose());
            });
        }

        inputs.estimatedPoses = new Pose2d[results.size()];
        inputs.timestamps = new double[results.size()];
        inputs.stdDeviations = new double[results.size()];

        for (int i = 0; i < results.size(); i++) {
            PoseAndTimestampAndDev entry = results.get(i);
            inputs.estimatedPoses[i] = entry.getPose();
            inputs.timestamps[i] = entry.getTimestamp();
            inputs.stdDeviations[i] = entry.getStdDev();
        }

        Logger.processInputs("Vision", inputs);
    }  

    public List<PoseAndTimestampAndDev> getResults() {
        return Collections.unmodifiableList(results);
    }

    public static class PoseAndTimestampAndDev implements Serializable {
        private static final long serialVersionUID = -1583699044845882985L;

        private final Pose2d pose;
        private final double timestamp;
        private final double stdDev;

        public PoseAndTimestampAndDev(Pose2d pose, double timestamp, double stdDev) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.stdDev = stdDev;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public double getStdDev() {
            return stdDev;
        }
    }

    public Camera[] getCameraArray() {
        return cameras;
    }

}
