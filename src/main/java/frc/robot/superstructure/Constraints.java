package frc.robot.superstructure;

import edu.wpi.first.math.util.Units;

/**
 * Centralised envelope and guard calculations for the superstructure.
 *
 * <p>The constants are intentionally conservative and should be tuned on-robot. All values are in
 * SI units.</p>
 */
public final class Constraints {
    private Constraints() {}

    private static final double MAX_ELEVATOR_HEIGHT = 1.25; // metres
    private static final double ELEVATOR_MIN_HEIGHT = 0.0;

    private static final double ELEVATOR_RETRACT_THRESHOLD = 0.10; // metres
    private static final double ELEVATOR_LIMIT_LOW_PIVOT = 0.15; // metres when pivot is near zero
    private static final double ELEVATOR_LIMIT_MID_PIVOT = 0.45; // metres when pivot is mid-range

    private static final double PIVOT_LOW_THRESHOLD = Units.degreesToRadians(18.0);
    private static final double PIVOT_MID_THRESHOLD = Units.degreesToRadians(38.0);
    private static final double PIVOT_HIGH_THRESHOLD = Units.degreesToRadians(58.0);

    private static final double PITCH_DOWN_LIMIT_LOW_PIVOT = Units.degreesToRadians(-15.0);
    private static final double PITCH_DOWN_LIMIT_MID_PIVOT = Units.degreesToRadians(-55.0);
    private static final double PITCH_DOWN_LIMIT_HIGH_PIVOT = Units.degreesToRadians(-100.0);

    private static final double REEF_CLEAR_HEIGHT = 0.42; // metres required to clear reef top
    private static final double REEF_APPROACH_PITCH_LIMIT = Units.degreesToRadians(-20.0);

    private static final double ROLL_MIN = Units.degreesToRadians(-200.0);
    private static final double ROLL_MAX = Units.degreesToRadians(200.0);

    private static final double PIVOT_HOLD_WINDOW = Units.degreesToRadians(2.0);

    public static boolean pivotMotionAllowed(double elevatorPositionMeters) {
        return elevatorPositionMeters < ELEVATOR_RETRACT_THRESHOLD;
    }

    public static double clampPivot(double desiredRadians, double currentRadians, double elevatorMeters) {
        if (pivotMotionAllowed(elevatorMeters)) {
            return desiredRadians;
        }
        // Hold current pivot while the elevator retracts.
        if (Math.abs(desiredRadians - currentRadians) <= PIVOT_HOLD_WINDOW) {
            return desiredRadians;
        }
        return currentRadians;
    }

    public static double clampElevator(
            double desiredMeters,
            double currentMeters,
            double pivotRadians,
            ManipulatorProfile profile
    ) {
        double allowable = MAX_ELEVATOR_HEIGHT;
        if (pivotRadians < PIVOT_LOW_THRESHOLD) {
            allowable = ELEVATOR_LIMIT_LOW_PIVOT;
        } else if (pivotRadians < PIVOT_MID_THRESHOLD) {
            allowable = ELEVATOR_LIMIT_MID_PIVOT;
        }
        allowable = Math.max(allowable, ELEVATOR_LIMIT_LOW_PIVOT);
        double commanded = clamp(desiredMeters, ELEVATOR_MIN_HEIGHT, allowable);

        // For climb profile we slow expansion when pivot is low to avoid bar interference.
        if (profile == ManipulatorProfile.CLIMB && pivotRadians < PIVOT_HIGH_THRESHOLD) {
            commanded = Math.min(commanded, currentMeters + 0.02);
        }

        return commanded;
    }

    public static double clampPitch(
            double desiredRadians,
            double pivotRadians,
            double elevatorMeters,
            ManipulatorProfile profile
    ) {
        double minPitchAllowed;
        if (pivotRadians < PIVOT_LOW_THRESHOLD) {
            minPitchAllowed = PITCH_DOWN_LIMIT_LOW_PIVOT;
        } else if (pivotRadians < PIVOT_MID_THRESHOLD) {
            minPitchAllowed = PITCH_DOWN_LIMIT_MID_PIVOT;
        } else {
            minPitchAllowed = PITCH_DOWN_LIMIT_HIGH_PIVOT;
        }

        if (elevatorMeters < REEF_CLEAR_HEIGHT) {
            minPitchAllowed = Math.max(minPitchAllowed, REEF_APPROACH_PITCH_LIMIT);
        }

        if (profile == ManipulatorProfile.CLIMB) {
            minPitchAllowed = Math.max(minPitchAllowed, Units.degreesToRadians(-35.0));
        }

        return Math.max(desiredRadians, minPitchAllowed);
    }

    public static double clampRoll(double desiredRadians) {
        return clamp(desiredRadians, ROLL_MIN, ROLL_MAX);
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
