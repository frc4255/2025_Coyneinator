package frc.robot.subsystems;

import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveIO.SwerveIOInputs;
import frc.robot.subsystems.Vision.VisionObservation;
import frc.robot.subsystems.Vision.VisionSubsystem;
import java.util.Arrays;
import java.util.Optional;

public class Swerve extends SubsystemBase {
    private final SwerveIO io;
    private final SwerveIOInputs inputs = new SwerveIOInputs();
    private final VisionSubsystem vision;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final DoubleArrayLogEntry moduleStatesLog;
    private final DoubleArrayLogEntry modulePositionsLog;
    private final DoubleArrayLogEntry gyroLog;

    private Optional<Pose2d> autoAlignTarget = Optional.empty();
    private static final int DASHBOARD_UPDATE_INTERVAL = 5;
    private static final int MODULE_LOG_INTERVAL = 2;
    private int dashboardCounter = 0;
    private int moduleLogCounter = 0;

    public Swerve(SwerveIO io, VisionSubsystem vision) {
        this.io = io;
        this.vision = vision;

        io.updateInputs(inputs);

        poseEstimator =
            new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                inputs.gyroYaw,
                inputs.modulePositions,
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.45, 0.45, 6)
            );

        DataLog log = DataLogManager.getLog();
        moduleStatesLog = new DoubleArrayLogEntry(log, SwerveIOInputs.MODULE_STATE_LOG_ENTRY);
        modulePositionsLog = new DoubleArrayLogEntry(log, SwerveIOInputs.MODULE_POSITION_LOG_ENTRY);
        gyroLog = new DoubleArrayLogEntry(log, SwerveIOInputs.GYRO_LOG_ENTRY);

        io.resetModulesToAbsolute();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        Rotation2d inverse = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
            ? new Rotation2d(Math.PI)
            : new Rotation2d();
        ChassisSpeeds requestedSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getHeading().rotateBy(inverse)
            )
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(requestedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxSpeed);
        io.setModuleStates(moduleStates, requestedSpeeds, isOpenLoop);
    }

    public void setAutoAlignTarget(Pose2d target) {
        autoAlignTarget = Optional.ofNullable(target);
    }

    public Optional<Pose2d> getAutoAlignTarget() {
        return autoAlignTarget;
    }

    public void clearAutoAlignTarget() {
        autoAlignTarget = Optional.empty();
    }

    private void follow(ChassisSpeeds speeds) {
        drive(
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            speeds.omegaRadiansPerSecond,
            true,
            false
        );
    }

    public void followTrajectory(SwerveSample sample) {
        if (sample == null) {
            return;
        }

        drive(new Translation2d(sample.vx, sample.vy), sample.omega, true, false);
    }

    public char findClosestBranch(boolean mirrorForRed) {
        return 'A';
    }


    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        ChassisSpeeds reference = Constants.Swerve.swerveKinematics.toChassisSpeeds(desiredStates);
        io.setModuleStates(desiredStates, reference, false);
    }

    public SwerveModuleState[] getModuleStates(){
        return Arrays.copyOf(inputs.moduleStates, inputs.moduleStates.length);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return inputs.chassisSpeeds;
    }

    public SwerveModulePosition[] getModulePositions(){
        return Arrays.copyOf(inputs.modulePositions, inputs.modulePositions.length);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(inputs.gyroYaw, getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        poseEstimator.resetPosition(inputs.gyroYaw, getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        io.zeroHeading();
        poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }


    /*
     * Gets the the gyro yaw and converts it to the robot coordinate plane (-180 to 180)
     */

    public Rotation2d getGyroYaw() {
        return inputs.gyroYaw;
    } //Why do they keep changing the API? They gotta make up their minds ong

    /* 
    this was used in 2024 code, is updated now

    public Rotation2d getGyroYaw() {
        double yaw = gyro.getAngle() % 360;

        if (yaw > 180) {
            yaw-=360;
        }

        return Rotation2d.fromDegrees(yaw*-1);
    }*/

    public void resetModulesToAbsolute(){
        io.resetModulesToAbsolute();
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        poseEstimator.update(inputs.gyroYaw, inputs.modulePositions);
        for (VisionObservation observation : vision.getResults()) {
            poseEstimator.addVisionMeasurement(
                observation.pose(),
                observation.timestamp()
                /*VecBuilder.fill(
                    stdDev,
                    stdDev,
                    5.0
                )*/
            );
        }

        moduleStatesLog.append(SwerveIOInputs.flattenModuleStates(inputs.moduleStates));
        modulePositionsLog.append(SwerveIOInputs.flattenModulePositions(inputs.modulePositions));
        gyroLog.append(new double[] {
            inputs.gyroYaw.getRadians(),
            inputs.gyroPitch.getRadians(),
            inputs.gyroRoll.getRadians()
        });

        dashboardCounter = (dashboardCounter + 1) % DASHBOARD_UPDATE_INTERVAL;
        if (dashboardCounter == 0) {
            SmartDashboard.putNumberArray(
                "Swerve/EstimatedPose",
                new Double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() }
            );
            SmartDashboard.putNumber("Swerve/EstimatedHeadingDegrees", inputs.gyroYaw.getDegrees());

            if (autoAlignTarget.isPresent()) {
                Pose2d target = autoAlignTarget.get();
                SmartDashboard.putNumberArray(
                    "Auto Align Target",
                    new Double[] { target.getX(), target.getY(), target.getRotation().getDegrees() }
                );
                SmartDashboard.putBoolean("Auto Align Active", true);
            } else {
                SmartDashboard.putBoolean("Auto Align Active", false);
            }
        }

        Pose2d currentPose = getPose();
        Logger.recordOutput("Swerve/EstimatedPose", Pose2d.struct, currentPose);
        Logger.recordOutput("Swerve/EstimatedHeadingDegrees", currentPose.getRotation().getDegrees());
        Logger.recordOutput(
            "Swerve/ChassisSpeeds",
            new double[] {
                inputs.chassisSpeeds.vxMetersPerSecond,
                inputs.chassisSpeeds.vyMetersPerSecond,
                inputs.chassisSpeeds.omegaRadiansPerSecond
            }
        );
        Logger.recordOutput("Swerve/AutoAlignActive", autoAlignTarget.isPresent());
        autoAlignTarget.ifPresent(target -> Logger.recordOutput("Swerve/AutoAlignTarget", Pose2d.struct, target));

        moduleLogCounter = (moduleLogCounter + 1) % MODULE_LOG_INTERVAL;
        if (moduleLogCounter == 0) {
            for (int i = 0; i < inputs.moduleStates.length; i++) {
                Logger.recordOutput(
                    "Swerve/Module" + i + "/SpeedMetersPerSecond",
                    inputs.moduleStates[i].speedMetersPerSecond
                );
                Logger.recordOutput(
                    "Swerve/Module" + i + "/AngleRadians",
                    inputs.moduleStates[i].angle.getRadians()
                );
            }

            for (int i = 0; i < inputs.modulePositions.length; i++) {
                Logger.recordOutput(
                    "Swerve/Module" + i + "/DistanceMeters",
                    inputs.modulePositions[i].distanceMeters
                );
            }
        }
    }
}



