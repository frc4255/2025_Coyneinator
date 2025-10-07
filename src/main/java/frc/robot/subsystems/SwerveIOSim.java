package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class SwerveIOSim implements SwerveIO {
    private static final double MAX_TURN_VOLTAGE = 6.0;

    private final SwerveModuleState[] moduleStates = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };
    private final double[] driveVoltages = new double[] {0.0, 0.0, 0.0, 0.0};
    private final double[] turnVoltages = new double[] {0.0, 0.0, 0.0, 0.0};

    private Rotation2d heading = new Rotation2d();
    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.02;
        }
        lastTimestamp = now;

        for (int i = 0; i < modulePositions.length; i++) {
            double distance = modulePositions[i].distanceMeters
                + moduleStates[i].speedMetersPerSecond * dt;
            modulePositions[i] = new SwerveModulePosition(
                distance,
                moduleStates[i].angle
            );

            inputs.drivePositionsMeters[i] = modulePositions[i].distanceMeters;
            inputs.turnPositionsRadians[i] = modulePositions[i].angle.getRadians();
            inputs.turnAbsoluteRadians[i] = modulePositions[i].angle.getRadians();
            inputs.driveVelocitiesMetersPerSecond[i] = moduleStates[i].speedMetersPerSecond;
            inputs.driveAppliedVolts[i] = driveVoltages[i];
            inputs.driveCurrentsAmps[i] = Math.abs(driveVoltages[i]) / 12.0 * 40.0;
            inputs.turnAppliedVolts[i] = turnVoltages[i];
            inputs.turnCurrentsAmps[i] = Math.abs(turnVoltages[i]) / 12.0 * 20.0;
        }

        ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(moduleStates);
        heading = heading.plus(new Rotation2d(speeds.omegaRadiansPerSecond * dt));

        inputs.gyroYawRadians = heading.getRadians();
        inputs.gyroPitchRadians = 0.0;
        inputs.gyroRollRadians = 0.0;
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        for (int i = 0; i < moduleStates.length && i < desiredStates.length; i++) {
            Rotation2d previousAngle = modulePositions[i].angle;
            moduleStates[i] = new SwerveModuleState(
                desiredStates[i].speedMetersPerSecond,
                desiredStates[i].angle
            );
            modulePositions[i] = new SwerveModulePosition(
                modulePositions[i].distanceMeters,
                desiredStates[i].angle
            );

            double normalizedSpeed = desiredStates[i].speedMetersPerSecond / Constants.Swerve.maxSpeed;
            normalizedSpeed = MathUtil.clamp(normalizedSpeed, -1.0, 1.0);
            driveVoltages[i] = normalizedSpeed * 12.0;

            // Approximate turn voltage based on angle error per cycle.
            double angleError = desiredStates[i].angle.minus(previousAngle).getRadians();
            turnVoltages[i] = MathUtil.clamp(angleError * 5.0, -MAX_TURN_VOLTAGE, MAX_TURN_VOLTAGE);
        }
    }

    @Override
    public void seedFieldRelative(Rotation2d heading) {
        this.heading = heading;
    }

    @Override
    public void resetToAbsolute() {
        for (int i = 0; i < modulePositions.length; i++) {
            modulePositions[i] = new SwerveModulePosition(0.0, modulePositions[i].angle);
        }
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        return modulePositions.clone();
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return moduleStates.clone();
    }
}
