package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Hardware abstraction for the swerve drivetrain. */
public interface SwerveIO {
    /** Shared telemetry structure populated by IO implementations. */
    public static class SwerveIOInputs {
        public static final int MODULE_COUNT = 4;

        public static final String MODULE_STATE_LOG_ENTRY = "/swerve/moduleStates";
        public static final String MODULE_POSITION_LOG_ENTRY = "/swerve/modulePositions";
        public static final String GYRO_LOG_ENTRY = "/swerve/gyro";

        public SwerveModuleState[] moduleStates = new SwerveModuleState[MODULE_COUNT];
        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[MODULE_COUNT];
        public Rotation2d gyroYaw = new Rotation2d();
        public Rotation2d gyroPitch = new Rotation2d();
        public Rotation2d gyroRoll = new Rotation2d();
        public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

        public SwerveIOInputs() {
            for (int i = 0; i < MODULE_COUNT; i++) {
                moduleStates[i] = new SwerveModuleState();
                modulePositions[i] = new SwerveModulePosition();
            }
        }

        public void setModuleStates(SwerveModuleState[] states) {
            for (int i = 0; i < MODULE_COUNT && i < states.length; i++) {
                moduleStates[i] = new SwerveModuleState(
                    states[i].speedMetersPerSecond,
                    states[i].angle
                );
            }
        }

        public void setModulePositions(SwerveModulePosition[] positions) {
            for (int i = 0; i < MODULE_COUNT && i < positions.length; i++) {
                modulePositions[i] = new SwerveModulePosition(
                    positions[i].distanceMeters,
                    positions[i].angle
                );
            }
        }

        public void copyFrom(SwerveIOInputs other) {
            setModuleStates(other.moduleStates);
            setModulePositions(other.modulePositions);
            gyroYaw = other.gyroYaw;
            gyroPitch = other.gyroPitch;
            gyroRoll = other.gyroRoll;
            chassisSpeeds = new ChassisSpeeds(
                other.chassisSpeeds.vxMetersPerSecond,
                other.chassisSpeeds.vyMetersPerSecond,
                other.chassisSpeeds.omegaRadiansPerSecond
            );
        }

        public static double[] flattenModuleStates(SwerveModuleState[] states) {
            double[] data = new double[states.length * 2];
            for (int i = 0; i < states.length; i++) {
                data[i * 2] = states[i].speedMetersPerSecond;
                data[i * 2 + 1] = states[i].angle.getRadians();
            }
            return data;
        }

        public static SwerveModuleState[] expandModuleStates(double[] data) {
            int count = data.length / 2;
            SwerveModuleState[] states = new SwerveModuleState[count];
            for (int i = 0; i < count; i++) {
                states[i] = new SwerveModuleState(
                    data[i * 2],
                    new Rotation2d(data[i * 2 + 1])
                );
            }
            return states;
        }

        public static double[] flattenModulePositions(SwerveModulePosition[] positions) {
            double[] data = new double[positions.length * 2];
            for (int i = 0; i < positions.length; i++) {
                data[i * 2] = positions[i].distanceMeters;
                data[i * 2 + 1] = positions[i].angle.getRadians();
            }
            return data;
        }

        public static SwerveModulePosition[] expandModulePositions(double[] data) {
            int count = data.length / 2;
            SwerveModulePosition[] positions = new SwerveModulePosition[count];
            for (int i = 0; i < count; i++) {
                positions[i] = new SwerveModulePosition(
                    data[i * 2],
                    new Rotation2d(data[i * 2 + 1])
                );
            }
            return positions;
        }
    }

    default void updateInputs(SwerveIOInputs inputs) {}

    default void setModuleStates(
        SwerveModuleState[] desiredStates,
        ChassisSpeeds referenceSpeeds,
        boolean isOpenLoop
    ) {}

    default void resetModulesToAbsolute() {}

    default void zeroHeading() {}

    default void stop() {}
}
