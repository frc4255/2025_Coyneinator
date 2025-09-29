package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveIOReal implements SwerveIO {
    private final Pigeon2 gyro;
    private final SwerveModule[] modules;

    public SwerveIOReal() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "Drivetrain");
        gyro.getConfigurator().apply(
            new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(180))
        );
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
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            SwerveModulePosition position = module.getPosition();
            SwerveModuleState state = module.getState();

            inputs.drivePositionsMeters[i] = position.distanceMeters;
            inputs.turnPositionsRadians[i] = position.angle.getRadians();
            inputs.turnAbsoluteRadians[i] = module.getCANcoder().getRadians();
            inputs.driveVelocitiesMetersPerSecond[i] = state.speedMetersPerSecond;
            inputs.driveAppliedVolts[i] = module.getDriveVoltage();
            inputs.driveCurrentsAmps[i] = module.getDriveCurrent();
            inputs.turnAppliedVolts[i] = module.getAngleVoltage();
            inputs.turnCurrentsAmps[i] = module.getAngleCurrent();
        }

        inputs.gyroYawRadians = Math.toRadians(gyro.getYaw().getValueAsDouble());
        inputs.gyroPitchRadians = Math.toRadians(gyro.getPitch().getValueAsDouble());
        inputs.gyroRollRadians = Math.toRadians(gyro.getRoll().getValueAsDouble());
    }

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop);
        }
    }

    @Override
    public void seedFieldRelative(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }

    @Override
    public void resetToAbsolute() {
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }
}
