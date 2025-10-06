package frc.robot.subsystems.Vision;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    /* Might need to create a custom class if I need more features. */
    private Camera[] cameras;

    /* Possibly a list of poses generated from each individual camera */
    public List<PoseAndTimestampAndDev> results = new ArrayList<>();

    public DoubleArrayLogEntry cameraPoseEntry;

    public VisionSubsystem(Camera[] cameras) {
        this.cameras = cameras;      
    }

    public byte[] convertToByteArray(Optional<VisionSubsystem.PoseAndTimestampAndDev> optionalPose) {
        if (optionalPose.isPresent()) {
            VisionSubsystem.PoseAndTimestampAndDev pose = optionalPose.get();
    
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
            cam.updateEstimate();
            cam.updateCameraPoseEntry();
            Optional<PoseAndTimestampAndDev> camEst = cam.getEstimate();
            if (camEst != null) {
                results.add(camEst.get());
                Logger.recordOutput("Camera " + cam + " Pose", camEst.get().getPose());
            }
        }
    }  

    public List<PoseAndTimestampAndDev> getResults() {
        return results;
    }

    public static class PoseAndTimestampAndDev {
        Pose2d pose;
        double timestamp;
        double stdDev;

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