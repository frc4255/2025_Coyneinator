package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.util.FlippingUtil;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.lang.annotation.Target;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import choreo.trajectory.SwerveSample;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem.PoseAndTimestampAndDev;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator m_SwervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private VisionSubsystem vision;

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(5, 0.0, 0.0);

    public Swerve(VisionSubsystem vision) {
        this.vision = vision;
        
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "Drivetrain");
        gyro.getConfigurator().apply(new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(180)));
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
        
        headingController.enableContinuousInput(-Math.PI, Math.PI);


        resetModulesToAbsolute();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        Rotation2d inverse = DriverStation.getAlliance().get() == Alliance.Red ?  new Rotation2d(Math.PI): new Rotation2d();
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() , 
                                    translation.getY(), 
                                    -rotation, 
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

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        Logger.recordOutput("Swerve Trajectory Sample", sample);
        // Apply the generated speeds
        follow(speeds);

        Logger.recordOutput("Speeds given to Swerve to follow", speeds);
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
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
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
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public char findClosestBranch(boolean flipForAlliance) {
        // Compute the angle from the reef center to the robot

        Translation2d allianceReef = flipForAlliance ? FlippingUtil.flipFieldPosition(FieldLayout.Reef.center) : FieldLayout.Reef.center;
        double dx = getPose().getX() - allianceReef.getX();
        double dy = getPose().getY() - allianceReef.getY();
        double angle = Math.toDegrees(Math.atan2(dy, dx));

        if (angle < 0) {
            angle += 360;
        }

        // Map angle to branch letters
        char[] branches = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L'};
        int sectorIndex = (int) (angle / 30.0);
        return branches[sectorIndex];
    }

    @Override
    public void periodic(){

        Pose2d currentPose = getPose();

        double[] array = {getPose().getX(), getPose().getY()};

        SmartDashboard.putNumberArray("Swerve Pose Estimation", array);

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
        
        //SmartDashboard.putNumberArray("Robot Pose", new Double[]{getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()});
        
        Logger.recordOutput("Robot Pose2d", getPose());
        Logger.recordOutput("Gyro angle", getGyroYaw().getDegrees());
        
        for (SwerveModule mod : mSwerveMods) {
            Logger.recordOutput("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            Logger.recordOutput("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            Logger.recordOutput("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        
        SmartDashboard.putNumber("Gyro angle", getGyroYaw().getDegrees());
    }
}