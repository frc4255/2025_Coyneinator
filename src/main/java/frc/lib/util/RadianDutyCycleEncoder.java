package frc.lib.util;


import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class RadianDutyCycleEncoder {
    private final DutyCycleEncoder encoder;
    private double offset;

    private static final double TWO_PI = 2.0 * Math.PI;

    public RadianDutyCycleEncoder(int channel) {
        encoder = new DutyCycleEncoder(channel);
        offset = 0.0;
    }


    public void setOffset(double offsetRadians) {
        offset = offsetRadians;
    }

    public double getPositionRadians() {

        double normalizedPosition = encoder.get();

        double positionRadians = normalizedPosition * TWO_PI;

        return normalizeRadians(positionRadians + offset);
    }

    private double normalizeRadians(double radians) {
        return (radians % TWO_PI + TWO_PI) % TWO_PI;
    }

    public void setDutyCycleRange(double min, double max) {
        encoder.setDutyCycleRange(min, max);
    }
}
