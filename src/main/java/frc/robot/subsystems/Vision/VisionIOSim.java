package frc.robot.subsystems.Vision;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class VisionIOSim implements VisionIO {
    private final Supplier<List<VisionObservation>> observationSupplier;

    public VisionIOSim() {
        this(Collections::emptyList);
    }

    public VisionIOSim(Supplier<List<VisionObservation>> observationSupplier) {
        this.observationSupplier = observationSupplier;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.setObservations(observationSupplier.get());
    }
}
