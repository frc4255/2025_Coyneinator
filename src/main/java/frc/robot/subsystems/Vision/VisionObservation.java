package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;

public record VisionObservation(Pose2d pose, double timestamp, double stdDev) {}
