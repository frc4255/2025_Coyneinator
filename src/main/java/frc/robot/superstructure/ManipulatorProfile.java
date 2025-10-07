package frc.robot.superstructure;

import java.util.Arrays;

/**
 * Profiles describe the default tolerances and settling behaviour for manipulator motion.
 * They allow the supervisor to request different aggressiveness for transit, scoring, or climb.
 */
public enum ManipulatorProfile {
    TRANSIT(new double[] {0.06, 0.04, 0.08, 0.12}, 0.0),
    SCORE(new double[] {0.04, 0.03, 0.05, 0.08}, 0.12),
    CLIMB(new double[] {0.05, 0.04, 0.06, 0.1}, 0.0);

    private final double[] defaultTolerance;
    private final double defaultHoldTime;

    ManipulatorProfile(double[] defaultTolerance, double defaultHoldTime) {
        this.defaultTolerance = Arrays.copyOf(defaultTolerance, defaultTolerance.length);
        this.defaultHoldTime = defaultHoldTime;
    }

    public double[] toleranceCopy() {
        return Arrays.copyOf(defaultTolerance, defaultTolerance.length);
    }

    public double defaultHoldTimeSeconds() {
        return defaultHoldTime;
    }
}
