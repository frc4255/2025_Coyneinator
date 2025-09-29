package frc.robot.subsystems;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

/**
 * Lightweight adapter that allows the {@link frc.robot.SubsystemManager}
 * to work with any mechanism that exposes a goal setter and an at-goal check.
 *
 * <p>The {@code key} should correspond to the entry in the mechanism graph's
 * node setpoints map (e.g. {@code "pivot"} or {@code "elevator"}).</p>
 */
public final class CoordinatedSubsystem {
    private final String key;
    private final DoubleConsumer goalConsumer;
    private final BooleanSupplier atGoalSupplier;

    public CoordinatedSubsystem(String key, DoubleConsumer goalConsumer, BooleanSupplier atGoalSupplier) {
        this.key = Objects.requireNonNull(key, "key");
        this.goalConsumer = Objects.requireNonNull(goalConsumer, "goalConsumer");
        this.atGoalSupplier = Objects.requireNonNull(atGoalSupplier, "atGoalSupplier");
    }

    public String key() {
        return key;
    }

    public void setGoal(double goal) {
        goalConsumer.accept(goal);
    }

    public boolean atGoal() {
        return atGoalSupplier.getAsBoolean();
    }
}
