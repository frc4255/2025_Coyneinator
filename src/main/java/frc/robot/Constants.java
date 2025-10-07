package frc.robot;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public final class Constants {
    public static final double stickDeadband = 0.1;

    public static class LEDs {

        public static HashMap<String, int[]> colorDatabase = new HashMap<String, int[]>();

        static {
            colorDatabase.put("Red", new int[] {255, 0, 0});
            colorDatabase.put("Green", new int[] {0, 255, 0});
            colorDatabase.put("Blue", new int[] {0, 0, 255});
            colorDatabase.put("Yellow", new int[] {255, 255, 0});
            colorDatabase.put("Purple", new int[] {255, 0, 255});
            colorDatabase.put("Cyan", new int[] {0, 255, 255});
            colorDatabase.put("White", new int[] {255, 255, 255});
            colorDatabase.put("noColor", new int[] {0, 0, 0});
        }
        
        public static enum LEDStates {
            OFF(10, colorDatabase.get("noColor"));


            
            private final int[] color;
            private final int priority;
    
            LEDStates(int priority, int[] color) {
                            this.priority = priority;
                            this.color = color;
            }
    
            public int getPriority() {
                return priority;
            }
            public int[] getColor() {
                return color;
            }
        }
    }

    public static final class PieceSensors {
        public static final int CORAL_INTAKE_CHANNEL = -1;
        public static final boolean CORAL_INTAKE_INVERTED = true;

        public static final int CORAL_WRIST_CHANNEL = -1;
        public static final boolean CORAL_WRIST_INVERTED = true;

        public static final int ALGAE_INTAKE_CHANNEL = -1;
        public static final boolean ALGAE_INTAKE_INVERTED = true;

        public static final int ALGAE_END_EFFECTOR_CHANNEL = -1;
        public static final boolean ALGAE_END_EFFECTOR_INVERTED = true;
    }

    public static final Map<Character, Pose2d> BLUE_REEF_SCORINGS_POSITIONS = new HashMap<>();

    static {
        BLUE_REEF_SCORINGS_POSITIONS.put('A', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('B', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('C', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('D', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('E', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('F', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('G', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('H', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('I', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('J', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('K', new Pose2d());
        BLUE_REEF_SCORINGS_POSITIONS.put('L', new Pose2d());
    }

    public final class Climber {
        public static final int MOTOR_ID = 7; //TODO change this maybe
    }
    public final class Wrist {
        public static final double kS = 0.0; //TODO tune this
        public static final double kG = 0.0; //TODO tune this 
        public static final double kV = 0.0; //TODO tune this  
        public static final double kA = 0.0; //TODO tune this 

        public static final double Pitch_kP = 0.05; //TODO tune this
        public static final double Roll_kP = 0.05; //TODO tune this

        /*Wrist Pitch range Limits && Roll range limits*/
        public static final double PitchMaxLimit = 0; //TODO tune this for RAD
        public static final double PitchMinLimit = 0; //TODO tune this for RAD

        public static final double RollMaxLimit = 0; //TODO tune this for RAD
        public static final double RollMinLimit = 0; //TODO tune this for RAD

        public static final int DIFFERENTIAL_LEFT_MOTOR_ID = 4;
        public static final int DIFFERENTIAL_RIGHT_MOTOR_ID = 5;
        public static final int DIFFERENTIAL_LEFT_ENCODER_ID = 14; // TODO configure actual ID
        public static final int DIFFERENTIAL_RIGHT_ENCODER_ID = 15; // TODO configure actual ID
        public static final double DIFFERENTIAL_MOTOR_TO_DIFF_GEAR_RATIO = 1.0; // TODO update: motor rotations per differential bevel rotation
        public static final double DIFFERENTIAL_PITCH_GEAR_RATIO = 51.0; // TODO update: differential rotations per wrist pitch rotation
        public static final double DIFFERENTIAL_ROLL_GEAR_RATIO = 60.0; // TODO update: differential rotations per wrist roll rotation

        public static final int END_EFFECTOR_MOTOR_ID = 6;
    }

    public final class PoseFilter {
        public static final double POSE_HEIGHT_TOLERANCE = 0.05;

        public static final double MAX_DIST_BETWEEN_POSE = 0.2;
    }

    public static final class Elevator {
        public static final double kP = 0.05; //Todo Tune this
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double MAX_VEL = 0;
        public static final double MAX_ACC = 0;
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final class Pivot {
            public static final double kS = 0; //TODO tune this 
            public static final double kG = 0; //TODO tune this
            public static final double kV = 0; //TODO tune this
            public static final double kA = 0; //TODO tune this
        }

        public static final int LEFT_MOTOR_ID = 2;
        public static final int RIGHT_MOTOR_ID = 3;

        public static final int PIVOT_LEFT_MOTOR_ID = 1;
        public static final int PIVOT_RIGHT_MOTOR_ID = 0;

        /*Pivot Max and Min limit */

        public static final double PivotMaxLimit = 0; //TODO tune this for RAD
        public static final double PivotMinLimit = 0; //TODO tune this for RAD

        public static final double ElevatorMaxExtensionLimit = 0; //TODO tune this in inches
        public static final double ElevatorMinExtensionLimit = 0; //This is actually 0

    }

    public static final class EndEffector {
        public static final double INTAKE_CORAL_VOLTS = 7.0;
        public static final double INTAKE_CORAL_HOLD_VOLTS = 2.0;
        public static final double HANDOFF_CORAL_VOLTS = 5.0;
        public static final double HOLD_CORAL_VOLTS = 1.0;
        public static final double OUTTAKE_CORAL_VOLTS = -4.5;
        public static final double OUTTAKE_CORAL_FAST_VOLTS = -7.0;

        public static final double INTAKE_ALGAE_VOLTS = 5.5;
        public static final double HOLD_ALGAE_VOLTS = 2.0;
        public static final double OUTTAKE_ALGAE_VOLTS = -6.0;

        public static final double CORAL_INTAKE_CURRENT_TRIGGER = 28.0;
        public static final double CORAL_INTAKE_CURRENT_LIMIT = 40.0;
        public static final double ALGAE_INTAKE_CURRENT_TRIGGER = 20.0;
    }

    public static final class Grabber2D {
        public static final double armLength = 0.0; //Todo get real value
        public static final double ElevatorminHeight = 0.0; //Todo get real value
        public static final double ElevatormaxHeight = 0.0; //Todo get real value

        public static final double wristLength = 0.0;

        public static final double floor = 0.0;
        public static final double maxVerticalRobotExtension = 90.389937; //TODO get real value approx whatever I put there (I didn't account for end effector height yet)
        public static final double maxHorizontalRobotExtension = 33.75; //Robot length / 2 + extension limit from frame perimeter.
    }


    public static final class GroundIntake {
        public static final int PITCH_LEADER_MOTOR_ID = -1;
        public static final int PITCH_FOLLOWER_MOTOR_ID = -1;
        public static final int ROLLER_MOTOR_ID = -1;
        public static final double PITCH_GEAR_RATIO = 1.0; // TODO: Update with the real ground intake gear ratio.
    }
    
    public static final class Swerve {

        public static final double ROBOT_MASS_KG = 64; //TODO get real value with bumpers and battery

        public static final double MAX_DRIVE_VELOCITY_MPS = 6.75; //TODO get real value
        public static final double WHEEL_COF = 1; //I got this one chief delphi so this might just be straight up wrong
        public static final double WHEEL_RADIUS_METERS = 0.0508; //TODO get real value
        public static final double DRIVE_MOTOR_CURRENT_LIMIT = 60; //TODO make sure this is right

        public static final Translation2d[] SWERVE_MODULE_LOCATIONS = {
                new Translation2d(Units.inchesToMeters(12.875), Units.inchesToMeters(10.375)),
                new Translation2d(Units.inchesToMeters(12.875), Units.inchesToMeters(-10.375)),
                new Translation2d(Units.inchesToMeters(-12.875), Units.inchesToMeters(10.375)),
                new Translation2d(Units.inchesToMeters(-12.875), Units.inchesToMeters(-10.375))
            };
        
        public static final double ROBOT_MOMENT_OF_INERTIA = 11; //TODO get real value in KG * M^2
        public static final int NUMBER_OF_MODULES = 4; 

        public static final double MAX_VELOCITY = 5; //TODO get real value
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelBase = Units.inchesToMeters(25.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-134.12);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-17.31);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(119.26);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

         /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-34.27);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
