package frc.robot.superstructure;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Wrapper around the E18-D80NK (and similar) sensors used for detection.
 */
public class PieceSensors implements AutoCloseable {
    private final Optional<DigitalInput> coralIntake;
    private final Optional<DigitalInput> coralWrist;
    private final Optional<DigitalInput> algaeEndEffector;

    public PieceSensors() {
        coralIntake = createInput(Constants.PieceSensors.CORAL_INTAKE_CHANNEL);
        coralWrist = createInput(Constants.PieceSensors.CORAL_WRIST_CHANNEL);
        algaeEndEffector = createInput(Constants.PieceSensors.ALGAE_END_EFFECTOR_CHANNEL);
    }

    private Optional<DigitalInput> createInput(int channel) {
        if (channel < 0) {
            return Optional.empty();
        }
        return Optional.of(new DigitalInput(channel));
    }

    public boolean hasCoralIntakeSensor() {
        return coralIntake.isPresent();
    }

    public boolean hasCoralWristSensor() {
        return coralWrist.isPresent();
    }

    public boolean hasAlgaeEndEffectorSensor() {
        return algaeEndEffector.isPresent();
    }

    public boolean coralDetectedInIntake() {
        return coralIntake.map(input -> read(input, Constants.PieceSensors.CORAL_INTAKE_INVERTED)).orElse(false);
    }

    public boolean coralDetectedAtWrist() {
        return coralWrist.map(input -> read(input, Constants.PieceSensors.CORAL_WRIST_INVERTED)).orElse(false);
    }

    public boolean algaeDetectedAtEndEffector() {
        return algaeEndEffector.map(input -> read(input, Constants.PieceSensors.ALGAE_END_EFFECTOR_INVERTED)).orElse(false);
    }

    private boolean read(DigitalInput input, boolean inverted) {
        boolean value = input.get();
        return inverted ? !value : value;
    }

    @Override
    public void close() {
        coralIntake.ifPresent(DigitalInput::close);
        coralWrist.ifPresent(DigitalInput::close);
        algaeEndEffector.ifPresent(DigitalInput::close);
    }
}
