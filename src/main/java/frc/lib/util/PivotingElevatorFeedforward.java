package frc.lib.util;

public class PivotingElevatorFeedforward {
    private final double kG;
    private final double kV;
    private final double kA;
    
    public PivotingElevatorFeedforward(double kG, double kV, double kA) {
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    public double calculate(double velocity, double acceleration, double angleRadians) {
        return kG * Math.sin(angleRadians) + kV * velocity + kA * acceleration;
    }
}