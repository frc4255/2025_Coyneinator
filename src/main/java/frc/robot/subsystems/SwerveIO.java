package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveIO {
    @AutoLog
    public static class SwerveIOInputs {
        public double[] drivePositionsMeters = new double[4];
        public double[] driveVelocitiesMetersPerSecond = new double[4];
        public double[] turnPositionsRadians = new double[4];
        public double[] turnAbsoluteRadians = new double[4];
        public double[] driveAppliedVolts = new double[4];
        public double[] driveCurrentsAmps = new double[4];
        public double[] turnAppliedVolts = new double[4];
        public double[] turnCurrentsAmps = new double[4];
        public double gyroYawRadians;
        public double gyroPitchRadians;
        public double gyroRollRadians;
    }

    default void updateInputs(SwerveIOInputs inputs) {}

    default void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {}

    default void seedFieldRelative(Rotation2d heading) {}

    default void resetToAbsolute() {}

    default SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {};
    }

    default SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {};
    }
}
