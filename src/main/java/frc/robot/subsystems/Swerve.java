package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.lang.annotation.Target;
import java.util.function.Consumer;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem.PoseAndTimestampAndDev;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator m_SwervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private VisionSubsystem vision;

    public Swerve(VisionSubsystem vision) {
        this.vision = vision;
        
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        Timer.delay(1);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
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

        Rotation2d inverse = DriverStation.getAlliance().get() == Alliance.Red ?  new Rotation2d(Math.PI): new Rotation2d();
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() , 
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

        for(SwerveModule mod : mSwerveMods){
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
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return m_SwervePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        m_SwervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }


    /* 
     * Gets the the gyro yaw and converts it to the robot coordinate plane (-180 to 180)
     */
    public Rotation2d getGyroYaw() {
        double yaw = gyro.getAngle() % 360;

        if (yaw > 180) {
            yaw-=360;
        }

        return Rotation2d.fromDegrees(yaw*-1);
    }

    /*public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }*/

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        m_SwervePoseEstimator.update(getGyroYaw(), getModulePositions());
        for (PoseAndTimestampAndDev poseAndTimestamp : vision.getResults()) {
            m_SwervePoseEstimator.addVisionMeasurement(
                poseAndTimestamp.getPose(),
                poseAndTimestamp.getTimestamp()
                /*VecBuilder.fill(
                    stdDev,
                    stdDev,
                    5.0
                )*/
            );
        }
        
        SmartDashboard.putNumberArray("Robot Pose", new Double[]{getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()});
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        
        SmartDashboard.putNumber("Gyro angle", getGyroYaw().getDegrees());
    }
}