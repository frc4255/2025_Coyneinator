package org.littletonrobotics.junction;

/**
 * Minimal stub of the AdvantageKit Logger so code can compile even when the
 * runtime library is not present. All methods no-op.
 */
public final class Logger {
    private Logger() {}

    public static void recordOutput(String key, double value) {}

    public static void recordOutput(String key, boolean value) {}
}
