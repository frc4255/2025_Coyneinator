package frc.robot.subsystems.Vision;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class VisionIOSim implements VisionIO {
    private Supplier<List<VisionObservation>> observationSupplier = Collections::emptyList;

    public VisionIOSim() {}

    public VisionIOSim(Supplier<List<VisionObservation>> observationSupplier) {
        setObservationSupplier(observationSupplier);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.setObservations(observationSupplier.get());
    }

    public void setObservationSupplier(Supplier<List<VisionObservation>> observationSupplier) {
        this.observationSupplier = observationSupplier != null ? observationSupplier : Collections::emptyList;
    }
}
