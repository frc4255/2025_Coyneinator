package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

public class VisionIOInputs {
    public final List<VisionObservation> observations = new ArrayList<>();

    public void setObservations(List<VisionObservation> newObservations) {
        observations.clear();
        observations.addAll(newObservations);
    }
}
