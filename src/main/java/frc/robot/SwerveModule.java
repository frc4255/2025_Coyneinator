package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    private DoubleLogEntry moduleSpeedLog;
    private DoubleLogEntry moduleAngleLog;
    private DoubleLogEntry encoderLog;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "Drivetrain");
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "Drivetrain");
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "Drivetrain");
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);



        moduleSpeedLog = new DoubleLogEntry(DataLogManager.getLog(), "swerve/module" + moduleNumber + "/speed");
        moduleAngleLog = new DoubleLogEntry(DataLogManager.getLog(), "swerve/module" + moduleNumber + "/angle");
        encoderLog = new DoubleLogEntry(DataLogManager.getLog(), "swerve/module" + moduleNumber + "/encoder");

        /* Angle Motor Config moved here so that data logging doesn't break*/
        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle);; //This will be removed next year which kinda breaks this file.
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);

        moduleSpeedLog.append(desiredState.speedMetersPerSecond);
        moduleAngleLog.append(desiredState.angle.getDegrees());
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        if (angleEncoder != null) {
            double absolutePosition = angleEncoder.getAbsolutePosition().getValueAsDouble();
            encoderLog.append(absolutePosition);
        }

        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }
    
    //public Rotation2d getCANcoder(){
    //    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    //}

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    //public SwerveModuleState getState(){
    //    return new SwerveModuleState(
    //        Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
    //        Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
    //    );
    //}

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    //public SwerveModulePosition getPosition(){
    //   return new SwerveModulePosition(
    //        Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
    //        Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
    //    );
    //}
}