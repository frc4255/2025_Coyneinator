package frc.lib.sim;

import frc.lib.sim.SwerveModuleSim; // Use your simulation-only module class
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Rotation;

import java.util.concurrent.Flow.Publisher;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem.PoseAndTimestampAndDev;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSim extends SubsystemBase {
    public SwerveDrivePoseEstimator m_SwervePoseEstimator;
    // Replace SwerveModule with your simulation-only version
    public SwerveModuleSim[] mSwerveMods;
    public Pigeon2 pigeon2D;
    public Pigeon2SimState gyro;

    private VisionSubsystem vision;

    private double rotation;

    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("SimSwerveModuleStates", SwerveModuleState.struct).publish(); //YES IT DOES THE CODE WILL BREAK IF YOU DELETE THIS

    private Pose2d simulatedPose = new Pose2d(); 

    public SwerveSim(VisionSubsystem vision) {
        this.vision = vision;
        
        // Instantiate the gyro and apply its simulation configuration if needed
        pigeon2D = new Pigeon2(Constants.Swerve.pigeonID);
        var gyro = pigeon2D.getSimState();
        gyro.addYaw(0);
        
        Timer.delay(1);

        // Create simulation-only swerve modules
        mSwerveMods = new SwerveModuleSim[] {
            new SwerveModuleSim(0, Constants.Swerve.Mod0.constants),
            new SwerveModuleSim(1, Constants.Swerve.Mod1.constants),
            new SwerveModuleSim(2, Constants.Swerve.Mod2.constants),
            new SwerveModuleSim(3, Constants.Swerve.Mod3.constants)
        };

        m_SwervePoseEstimator =
            new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.45, 0.45, 6)
            );

        resetModulesToAbsolute();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Depending on alliance, mirror the heading if necessary.
        Rotation2d inverse = DriverStation.getAlliance().get() == Alliance.Red ? new Rotation2d(Math.PI) : new Rotation2d();
        
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading().rotateBy(inverse)
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        // Command each simulation-only module.
        for(SwerveModuleSim mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
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

        PathFollowingController holonomicController = new PathFollowingController() {

            @Override
            public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
                Translation2d positionError = targetState.pose.getTranslation().minus(currentPose.getTranslation());
                Rotation2d rotationError = targetState.pose.getRotation().minus(currentPose.getRotation());
        
                // Basic proportional control
                double vx = positionError.getX() * Constants.Swerve.driveKP;
                double vy = positionError.getY() * Constants.Swerve.driveKP;
                double omega = rotationError.getRadians() * Constants.Swerve.driveKP;
        
                return new ChassisSpeeds(vx, vy, omega);
            }

            @Override
            public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
                // No reset behavior for now.
            }

            @Override
            public boolean isHolonomic() {
                return true;
            }
        };

        return new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier (robot-relative)
                (speeds, feedforwards) -> {
                    // Drive the robot using the calculated speeds
                    drive(
                        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                        speeds.omegaRadiansPerSecond,
                        false,
                        false
                    );
                },
                holonomicController,
                Constants.Swerve.robotConfig,
                () -> {
                    // Determine if the path should be mirrored based on alliance.
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this // Subsystem requirement
        );
    }

    /* Used by SwerveControllerCommand in Autonomous */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModuleSim mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModuleSim mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModuleSim mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return simulatedPose;
    }

    public void setPose(Pose2d pose) {
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return simulatedPose.getRotation();
    }

    public void setHeading(Rotation2d heading){
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
     * Gets the gyro yaw angle converted to the robot coordinate system (-180 to 180 degrees).
     */
    public Rotation2d getGyroYaw() {

        //gyro.setRawYaw(6);
        //return Rotation2d.fromDegrees(-yaw);
        return Rotation2d.fromDegrees(0);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModuleSim mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void updateGyroSim(double rotationSpeed) {
        if (gyro != null) {
            double angularVelocity = rotationSpeed * Constants.Swerve.maxAngularVelocity / 4; // Scale as needed

            gyro.setAngularVelocityZ(angularVelocity);
    
            double deltaYaw = angularVelocity * 0.02; // 20ms loop
            gyro.addYaw(deltaYaw);
        }
    }

    private void updateSimulatedPose() {
        double dt = 0.02; // 20ms timestep

        ChassisSpeeds robotSpeeds = getChassisSpeeds();

        Rotation2d currentHeading = simulatedPose.getRotation();
        double cosAngle = Math.cos(currentHeading.getRadians());
        double sinAngle = Math.sin(currentHeading.getRadians());

        double fieldVx = robotSpeeds.vxMetersPerSecond * cosAngle - robotSpeeds.vyMetersPerSecond * sinAngle;
        double fieldVy = robotSpeeds.vxMetersPerSecond * sinAngle + robotSpeeds.vyMetersPerSecond * cosAngle;

        Translation2d deltaTranslation = new Translation2d(fieldVx * dt, fieldVy * dt);
        Rotation2d deltaRotation = Rotation2d.fromRadians(robotSpeeds.omegaRadiansPerSecond * dt * dt);

        Translation2d newTranslation = simulatedPose.getTranslation().plus(deltaTranslation);
        Rotation2d newRotation = simulatedPose.getRotation().plus(deltaRotation);
        simulatedPose = new Pose2d(newTranslation, newRotation);
    }

    

    @Override
    public void periodic(){
        // Update the pose estimator
        m_SwervePoseEstimator.update(getGyroYaw(), getModulePositions());
        
        // Incorporate vision measurements if available.
        for (PoseAndTimestampAndDev poseAndTimestamp : vision.getResults()) {
            m_SwervePoseEstimator.addVisionMeasurement(
                poseAndTimestamp.getPose(),
                poseAndTimestamp.getTimestamp()
            );
        }
        
        SmartDashboard.putNumberArray("Robot Pose", new Double[]{
            getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()
        });
        
        // Publish module information
        for(SwerveModuleSim mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        publisher.set(getModuleStates());

        updateGyroSim(rotation);

        updateSimulatedPose();
    }
}
