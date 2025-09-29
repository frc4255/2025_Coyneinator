package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveIO.SwerveIOInputs;
import frc.robot.subsystems.Vision.VisionObservation;
import frc.robot.subsystems.Vision.VisionSubsystem;
import java.util.Arrays;
import java.util.Optional;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

public class Swerve extends SubsystemBase {
    private final SwerveIO io;
    private final SwerveIOInputs inputs = new SwerveIOInputs();
    private final VisionSubsystem vision;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final DoubleArrayLogEntry moduleStatesLog;
    private final DoubleArrayLogEntry modulePositionsLog;
    private final DoubleArrayLogEntry gyroLog;

    private Optional<Pose2d> autoAlignTarget = Optional.empty();

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
            false,
            false
        );
    }

    public Command followPathCommand(PathPlannerPath path) {

        PathFollowingController HolonomicController = new PathFollowingController() {

            @Override
            public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
                Translation2d positionError = targetState.pose.getTranslation().minus(currentPose.getTranslation());
                Rotation2d rotationError = targetState.pose.getRotation().minus(currentPose.getRotation());
        
                // Apply basic proportional control (adjust gains as needed)
                double vx = positionError.getX() * Constants.Swerve.driveKP;
                double vy = positionError.getY() * Constants.Swerve.driveKP;
                double omega = rotationError.getRadians() * Constants.Swerve.driveKP;
        
                return new ChassisSpeeds(vx, vy, omega);

            }

            @Override
            public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
                
            }

            @Override
            public boolean isHolonomic() {
                return true;
            }
            
            
        };

        return new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> {
                    // Drive the robot using calculated ChassisSpeeds and feedforwards
                    drive(
                        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                        speeds.omegaRadiansPerSecond,
                        false,
                        false
                    );}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                HolonomicController, // PathFollowerController
                Constants.Swerve.robotConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
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

        SmartDashboard.putNumberArray(
            "Robot Pose",
            new Double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() }
        );
        SmartDashboard.putNumber("Gyro angle", inputs.gyroYaw.getDegrees());

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
}