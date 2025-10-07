package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** Lightweight simulation implementation of {@link SwerveIO}. */
public class SwerveIOSim implements SwerveIO {

    private static final class ModuleSim {
        final int moduleNumber;
        private SwerveModuleState state = new SwerveModuleState();
        private SwerveModulePosition position = new SwerveModulePosition();

        ModuleSim(int moduleNumber) {
            this.moduleNumber = moduleNumber;
        }

        void setDesiredState(SwerveModuleState desiredState) {
            state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }

        void update(double dtSeconds) {
            position = new SwerveModulePosition(
                position.distanceMeters + state.speedMetersPerSecond * dtSeconds,
                state.angle
            );
        }

        void reset() {
            position = new SwerveModulePosition(0.0, new Rotation2d());
            state = new SwerveModuleState();
        }

        SwerveModuleState getState() {
            return state;
        }

        SwerveModulePosition getPosition() {
            return position;
        }
    }

    private final ModuleSim[] modules = new ModuleSim[] {
        new ModuleSim(0),
        new ModuleSim(1),
        new ModuleSim(2),
        new ModuleSim(3)
    };

    private Rotation2d simulatedYaw = new Rotation2d();
    private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        for (ModuleSim module : modules) {
            module.update(dt);
        }

        simulatedYaw = simulatedYaw.plus(new Rotation2d(commandedSpeeds.omegaRadiansPerSecond * dt));

        inputs.setModuleStates(getModuleStates());
        inputs.setModulePositions(getModulePositions());
        inputs.gyroYaw = simulatedYaw;
        inputs.gyroPitch = new Rotation2d();
        inputs.gyroRoll = new Rotation2d();
        inputs.chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(inputs.moduleStates);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates, ChassisSpeeds referenceSpeeds, boolean isOpenLoop) {
        commandedSpeeds = referenceSpeeds;
        for (int i = 0; i < modules.length && i < desiredStates.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    @Override
    public void resetModulesToAbsolute() {
        for (ModuleSim module : modules) {
            module.reset();
        }
    }

    @Override
    public void zeroHeading() {
        simulatedYaw = new Rotation2d();
    }

    @Override
    public void stop() {
        commandedSpeeds = new ChassisSpeeds();
        for (ModuleSim module : modules) {
            module.setDesiredState(new SwerveModuleState());
        }
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (ModuleSim module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (ModuleSim module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }
}
