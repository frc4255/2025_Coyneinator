package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.SwerveModule;

/** Real-hardware implementation of {@link SwerveIO}. */
public class SwerveIOTalonFX implements SwerveIO {
    private final SwerveModule[] modules;
    private final Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID);

    public SwerveIOTalonFX() {
        gyro.getConfigurator().apply(new com.ctre.phoenix6.configs.Pigeon2Configuration());
        gyro.setYaw(0.0);

        Timer.delay(1.0);

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.setModuleStates(getModuleStates());
        inputs.setModulePositions(getModulePositions());
        inputs.gyroYaw = Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
        inputs.gyroPitch = Rotation2d.fromDegrees(gyro.getPitch().getValueAsDouble());
        inputs.gyroRoll = Rotation2d.fromDegrees(gyro.getRoll().getValueAsDouble());
        inputs.chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(inputs.moduleStates);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates, ChassisSpeeds referenceSpeeds, boolean isOpenLoop) {
        for (int i = 0; i < modules.length && i < desiredStates.length; i++) {
            modules[i].setDesiredState(desiredStates[i], isOpenLoop);
        }
    }

    @Override
    public void resetModulesToAbsolute() {
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    @Override
    public void zeroHeading() {
        gyro.setYaw(0.0);
    }

    @Override
    public void stop() {
        for (SwerveModule module : modules) {
            module.setDesiredState(new SwerveModuleState(), true);
        }
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }
}
