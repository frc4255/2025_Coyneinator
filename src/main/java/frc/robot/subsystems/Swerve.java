package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FlippingUtil;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.subsystems.SwerveIOInputsAutoLogged;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem.PoseAndTimestampAndDev;

public class Swerve extends SubsystemBase {
    private final SwerveIO io;
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
    private final VisionSubsystem vision;

    public final SwerveDrivePoseEstimator m_SwervePoseEstimator;

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(5, 0.0, 0.0);

    public Swerve(SwerveIO io, VisionSubsystem vision) {
        this.io = io;
        this.vision = vision;

        io.updateInputs(inputs);

        m_SwervePoseEstimator =
            new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.45, 0.45, 6)
            );

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        io.resetToAbsolute();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        Rotation2d allianceRotation = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? new Rotation2d(Math.PI)
            : new Rotation2d();

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        -rotation,
                        getHeading().rotateBy(allianceRotation)
                    )
                    : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation
                    )
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        io.setModuleStates(swerveModuleStates, isOpenLoop);
    }

    private void follow(ChassisSpeeds speeds) {
        drive(
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            speeds.omegaRadiansPerSecond,
            false,
            false
        );
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        Logger.recordOutput("Swerve/TrajectorySample", sample);
        follow(speeds);
        Logger.recordOutput("Swerve/CommandedSpeeds", speeds);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        io.setModuleStates(desiredStates, false);
    }

    public SwerveModuleState[] getModuleStates() {
        return io.getModuleStates();
    }

    public SwerveModulePosition[] getModulePositions() {
        return io.getModulePositions();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return m_SwervePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        io.seedFieldRelative(heading);
        m_SwervePoseEstimator.resetPosition(
            heading,
            getModulePositions(),
            new Pose2d(getPose().getTranslation(), heading)
        );
    }

    public void zeroHeading() {
        Rotation2d heading = new Rotation2d();
        io.seedFieldRelative(heading);
        m_SwervePoseEstimator.resetPosition(
            heading,
            getModulePositions(),
            new Pose2d(getPose().getTranslation(), heading)
        );
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromRadians(inputs.gyroYawRadians);
    }

    public void resetModulesToAbsolute() {
        io.resetToAbsolute();
    }

    public char findClosestBranch(boolean flipForAlliance) {
        Translation2d allianceReef = flipForAlliance
            ? FlippingUtil.flipFieldPosition(FieldLayout.Reef.center)
            : FieldLayout.Reef.center;
        double dx = getPose().getX() - allianceReef.getX();
        double dy = getPose().getY() - allianceReef.getY();
        double angle = Math.toDegrees(Math.atan2(dy, dx));

        if (angle < 0) {
            angle += 360;
        }

        angle = (angle + 180) % 360;

        char[] branches = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L'};
        int sectorIndex = (int) (angle / 30.0);

        Logger.recordOutput("Swerve/ActiveSector", branches[sectorIndex]);

        return branches[sectorIndex];
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve", inputs);

        m_SwervePoseEstimator.update(getGyroYaw(), getModulePositions());
        for (PoseAndTimestampAndDev poseAndTimestamp : vision.getResults()) {
            m_SwervePoseEstimator.addVisionMeasurement(
                poseAndTimestamp.getPose(),
                poseAndTimestamp.getTimestamp()
            );
        }

        Logger.recordOutput("Swerve/Pose", getPose());
        Logger.recordOutput("Swerve/GyroYawDegrees", getGyroYaw().getDegrees());
        Logger.recordOutput("Swerve/ChassisSpeeds", getChassisSpeeds());
        Logger.recordOutput("Swerve/ModuleStates", getModuleStates());
    }
}
