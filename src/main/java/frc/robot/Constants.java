package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    private Constants() {}

    public static final double stickDeadband = 0.1;

    public static final class GroundIntake {
        private GroundIntake() {}

        public static final int PITCH_LEADER_MOTOR_ID = 7;
        public static final int PITCH_FOLLOWER_MOTOR_ID = 8;
        public static final int ROLLER_MOTOR_ID = 9;

        public static final int CORAL_SENSOR_ROBORIO_DIGITAL_CHANNEL = 0;
        public static final double PITCH_GEAR_RATIO = 1.0;

        public static final double ZERO_OFFSET_RADIANS = 0.0;

        public static final double PITCH_KS = 0.0;
        public static final double PITCH_KG = 0.0;
        public static final double PITCH_KV = 0.0;
        public static final double PITCH_KA = 0.0;

        public static final double PITCH_KP = 1.0;
        public static final double PITCH_KI = 0.0;
        public static final double PITCH_KD = 0.0;

        public static final double PITCH_MAX_VELOCITY_RAD_PER_SEC = 4.0;
        public static final double PITCH_MAX_ACCEL_RAD_PER_SEC_SQ = 10.0;
        public static final double PITCH_POSITION_TOLERANCE_RADIANS = Math.toRadians(2.0);
        public static final double PITCH_VELOCITY_TOLERANCE_RAD_PER_SEC = Units.degreesToRadians(5.0);

        public static final double DEFAULT_PITCH_RADIANS = 0.0;
        public static final double MIN_PITCH_RADIANS = Units.degreesToRadians(-90.0);
        public static final double MAX_PITCH_RADIANS = Units.degreesToRadians(90.0);

        public static final Translation3d VISUALIZER_BASE_MOUNT = new Translation3d(0.333, 0.0, 0.220);
        public static final Translation3d VISUALIZER_ROLLER_OFFSET = new Translation3d(0.25, 0.0, 0.05);
    }

    public static final class Swerve {
        private Swerve() {}

        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule =
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(
                COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2
            );

        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelBase = Units.inchesToMeters(25.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        public static final Translation2d[] SWERVE_MODULE_LOCATIONS = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        public static final double maxSpeed = 4.5;
        public static final double maxAngularVelocity = 10.0;

        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final boolean angleEnableCurrentLimit = true;
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;

        public static final boolean driveEnableCurrentLimit = true;
        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;

        public static final class Mod0 {
            private Mod0() {}
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-34.27);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final class Mod1 {
            private Mod1() {}
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(119.26);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final class Mod2 {
            private Mod2() {}
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-17.31);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final class Mod3 {
            private Mod3() {}
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-134.12);
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
