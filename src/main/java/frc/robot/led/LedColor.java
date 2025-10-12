package frc.robot.led;

/**
 * Simple RGB color wrapper used by LED patterns.
 */
public record LedColor(int r, int g, int b) {
    public static final LedColor BLACK = new LedColor(0, 0, 0);

    public LedColor {
        r = clamp(r);
        g = clamp(g);
        b = clamp(b);
    }

    private static int clamp(int value) {
        return Math.max(0, Math.min(255, value));
    }

    public int[] toArray() {
        return new int[] {r, g, b};
    }

    @Override
    public String toString() {
        return "LedColor{" + r + "," + g + "," + b + "}";
    }
}
