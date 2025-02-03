package frc.lib.sim;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModuleSim {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private TalonFXSimState driveSimState;
    private TalonFXSimState angleSimState;
    private CANcoderSimState encoderSimState;

    private final SimpleMotorFeedforward driveFeedForward =
        new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    private double simulatedDriveVelocity = 0;   // [rotations per second]
    private double simulatedDrivePosition = 0;     // [rotations]
    private double simulatedAnglePosition = 0;     // [rotations]

    private final PositionVoltage anglePositionCommand = new PositionVoltage(0);

    public SwerveModuleSim(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);

        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        driveSimState = mDriveMotor.getSimState();
        angleSimState = mAngleMotor.getSimState();
        encoderSimState = angleEncoder.getSimState();

        driveSimState.setSupplyVoltage(12);
        angleSimState.setSupplyVoltage(12);
        encoderSimState.setSupplyVoltage(12);

        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        double desiredAngleRotations = desiredState.angle.getRotations();
        angleSimState.setRawRotorPosition(desiredAngleRotations);
        simulatedAnglePosition = desiredAngleRotations;

        double targetRPS = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
        driveSimState.setRotorVelocity(targetRPS);
        simulatedDriveVelocity = targetRPS;
    }

    public Rotation2d getCANcoder() {

        return Rotation2d.fromRotations(simulatedAnglePosition);
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        angleSimState.setRawRotorPosition(absolutePosition);
        simulatedAnglePosition = absolutePosition;
    }

 
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(simulatedDriveVelocity, Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(simulatedAnglePosition)
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(simulatedDrivePosition, Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(simulatedAnglePosition)
        );
    }

    public void updateSim() {
        double dt = 0.02; // 20 ms timestep

        double batteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage();
        RoboRioSim.setVInVoltage(batteryVoltage);

        driveSimState.addRotorPosition(simulatedDriveVelocity * dt);
        simulatedDrivePosition += simulatedDriveVelocity * dt;

        // For the angle motor, we assume the simulation state is set immediately,
        // so no incremental update is required.
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("poseOfSwervemodule" + moduleNumber + " Drive Motor", getPosition());

    }
}
