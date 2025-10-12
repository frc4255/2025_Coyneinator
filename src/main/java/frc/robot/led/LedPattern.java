package frc.robot.led;

import java.util.Objects;

/**
 * Describes a prioritized LED pattern request.
 */
public record LedPattern(
        int priority,
        Style style,
        LedColor primary,
        LedColor secondary,
        double periodSeconds
) {
    private static final double DEFAULT_PERIOD_SECONDS = 0.5;

    public enum Style {
        SOLID,
        FLASHING,
        ALTERNATING
    }

    public LedPattern {
        Objects.requireNonNull(style, "style");
        Objects.requireNonNull(primary, "primary");
        Objects.requireNonNull(secondary, "secondary");
        if (priority < 0) {
            priority = 0;
        }
        if (periodSeconds <= 0.0 && style != Style.SOLID) {
            periodSeconds = DEFAULT_PERIOD_SECONDS;
        }
    }

    public static LedPattern solid(int priority, LedColor color) {
        return new LedPattern(priority, Style.SOLID, color, color, 0.0);
    }

    public static LedPattern flashing(int priority, LedColor color, double periodSeconds) {
        return new LedPattern(priority, Style.FLASHING, color, LedColor.BLACK, periodSeconds);
    }

    public static LedPattern alternating(int priority, LedColor first, LedColor second, double periodSeconds) {
        return new LedPattern(priority, Style.ALTERNATING, first, second, periodSeconds);
    }
}
