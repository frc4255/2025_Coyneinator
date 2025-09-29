package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.sim.SwerveModuleSim;
import frc.robot.Constants;

/** Physics-based simulation implementation of {@link SwerveIO}. */
public class SwerveIOSim implements SwerveIO {
    private final SwerveModuleSim[] modules = new SwerveModuleSim[] {
        new SwerveModuleSim(0, Constants.Swerve.Mod0.constants),
        new SwerveModuleSim(1, Constants.Swerve.Mod1.constants),
        new SwerveModuleSim(2, Constants.Swerve.Mod2.constants),
        new SwerveModuleSim(3, Constants.Swerve.Mod3.constants)
    };

    private Rotation2d simulatedYaw = new Rotation2d();
    private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
    private double lastTimestamp = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        for (SwerveModuleSim module : modules) {
            module.updateSim();
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
            modules[i].setDesiredState(desiredStates[i], isOpenLoop);
        }
    }

    @Override
    public void resetModulesToAbsolute() {
        for (SwerveModuleSim module : modules) {
            module.resetToAbsolute();
        }
    }

    @Override
    public void zeroHeading() {
        simulatedYaw = new Rotation2d();
    }

    @Override
    public void stop() {
        commandedSpeeds = new ChassisSpeeds();
        for (SwerveModuleSim module : modules) {
            module.setDesiredState(new SwerveModuleState(), true);
        }
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (SwerveModuleSim module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (SwerveModuleSim module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }
}
