package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.StateManager;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.lib.util.graph.GraphParser;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOPhoton;
import frc.robot.subsystems.Vision.VisionIOPhoton.CameraConfig;
import frc.robot.subsystems.Vision.VisionIOSim;
import frc.robot.subsystems.Vision.VisionSubsystem;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private static final List<CameraConfig> VISION_CAMERA_CONFIGS = List.of(
        new CameraConfig(
            "LeftCam",
            new Transform3d(
                new Translation3d(0.258, 0.291, 0.2),
                new Rotation3d(0, -1.08, 0.523)
            )
        ),
        new CameraConfig(
            "RightCam",
            new Transform3d(
                new Translation3d(0.258, -0.291, 0.2),
                new Rotation3d(0, -1.08, -0.523)
            )
        )
    );

    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton toggleRobotState = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final VisionSubsystem s_VisionSubystem = new VisionSubsystem(createVisionIO());

    private enum SwerveIOMode {
        AUTO,
        REAL,
        SIM,
        REPLAY
    }

    private enum VisionIOMode {
        AUTO,
        PHOTON,
        SIM
    }

    private final Swerve s_Swerve = new Swerve(createSwerveIO(), s_VisionSubystem);
    private final AutoAlignController autoAlignController = createAutoAlignController();

    private final Pivot s_Pivot = new Pivot(
        RobotBase.isReal() ? new PivotIOTalonFX() : new PivotIOSim()
    );
    private final Elevator s_Elevator = new Elevator(
        RobotBase.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim(),
        s_Pivot::getPivotPosition
    );
    private final WristPitch s_WristPitch = new WristPitch(
        RobotBase.isReal() ? new WristPitchIOTalonFX() : new WristPitchIOSim()
    );
    private final WristRoll s_WristRoll = new WristRoll(
        RobotBase.isReal() ? new WristRollIOTalonFX() : new WristRollIOSim()
    );

    private AutoAlignController createAutoAlignController() {
        AutoAlignController controller = new AutoAlignController(
            new Joystick(Constants.OperatorConstants.AUTO_ALIGN_JOYSTICK_PORT),
            loadAutoAlignConfig(),
            StateManager::getCurrentState
        );
        controller.setPoseListener(s_Swerve::setAutoAlignTarget);
        return controller;
    }

    private AutoAlignConfig loadAutoAlignConfig() {
        String preset = System.getProperty("autoAlignPreset");
        if (preset == null || preset.isBlank()) {
            preset = System.getenv("AUTO_ALIGN_PRESET");
        }

        if (preset == null || preset.isBlank()) {
            return AutoAlignConfig.empty();
        }

        if ("orientation".equalsIgnoreCase(preset.trim())) {
            return AutoAlignPresets.orientation();
        }

        System.err.println("Unrecognized auto align preset '" + preset + "', defaulting to empty configuration.");
        return AutoAlignConfig.empty();
    }

    private VisionIO createVisionIO() {
        VisionIOMode mode = getRequestedVisionIOMode();
        if (mode == VisionIOMode.AUTO) {
            return RobotBase.isReal()
                ? new VisionIOPhoton(VISION_CAMERA_CONFIGS)
                : new VisionIOSim();
        }

        switch (mode) {
            case PHOTON:
                return new VisionIOPhoton(VISION_CAMERA_CONFIGS);
            case SIM:
            default:
                return new VisionIOSim();
        }
    }

    private VisionIOMode getRequestedVisionIOMode() {
        String modeValue = System.getProperty("visionIOMode");
        if (modeValue == null || modeValue.isBlank()) {
            modeValue = System.getenv("VISION_IO_MODE");
        }

        if (modeValue == null || modeValue.isBlank()) {
            return VisionIOMode.AUTO;
        }

        try {
            return VisionIOMode.valueOf(modeValue.trim().toUpperCase());
        } catch (IllegalArgumentException ex) {
            System.err.println("Unrecognized vision IO mode '" + modeValue + "', defaulting to AUTO.");
            return VisionIOMode.AUTO;
        }
    }

    private SwerveIO createSwerveIO() {
        SwerveIOMode requestedMode = getRequestedSwerveIOMode();

        if (RobotBase.isReal()) {
            if (requestedMode == SwerveIOMode.SIM || requestedMode == SwerveIOMode.REPLAY) {
                System.out.println("Ignoring requested swerve IO mode '" + requestedMode + "' because the robot is running on real hardware.");
            }
            return new SwerveIOTalonFX();
        }

        switch (requestedMode) {
            case REAL:
                return new SwerveIOTalonFX();
            case SIM:
                return new SwerveIOSim();
            case REPLAY: {
                SwerveIO replay = attemptLoadReplay();
                if (replay != null) {
                    return replay;
                }
                System.err.println("Falling back to SwerveIOSim after replay log load failed.");
                return new SwerveIOSim();
            }
            case AUTO:
            default: {
                SwerveIO replay = attemptLoadReplay();
                return replay != null ? replay : new SwerveIOSim();
            }
        }
    }

    private SwerveIOMode getRequestedSwerveIOMode() {
        String modeValue = System.getProperty("swerveIOMode");
        if (modeValue == null || modeValue.isBlank()) {
            modeValue = System.getenv("SWERVE_IO_MODE");
        }

        if (modeValue == null || modeValue.isBlank()) {
            return SwerveIOMode.AUTO;
        }

        try {
            return SwerveIOMode.valueOf(modeValue.trim().toUpperCase());
        } catch (IllegalArgumentException ex) {
            System.err.println("Unrecognized swerve IO mode '" + modeValue + "', defaulting to AUTO.");
            return SwerveIOMode.AUTO;
        }
    }

    private SwerveIO attemptLoadReplay() {
        String replayPath = System.getProperty("swerveReplayLog");
        if (replayPath == null || replayPath.isBlank()) {
            replayPath = System.getenv("SWERVE_REPLAY_LOG");
        }

        if (replayPath == null || replayPath.isBlank()) {
            return null;
        }

        Path logPath = Path.of(replayPath);
        if (Files.exists(logPath)) {
            try {
                return new SwerveIOReplay(logPath);
            } catch (RuntimeException ex) {
                System.err.println("Failed to load swerve replay log from '" + logPath + "': " + ex.getMessage());
            }
        } else {
            System.err.println("Swerve replay log '" + logPath + "' does not exist.");
        }

        return null;
    }

    private final SubsystemManager subsystemManager = new SubsystemManager(
        new CoordinatedSubsystem(SubsystemManager.PIVOT_KEY, s_Pivot::setGoal, s_Pivot::atGoal),
        new CoordinatedSubsystem(SubsystemManager.ELEVATOR_KEY, s_Elevator::setGoal, s_Elevator::atGoal),
        new CoordinatedSubsystem(SubsystemManager.WRIST_PITCH_KEY, s_WristPitch::setGoal, s_WristPitch::atGoal),
        new CoordinatedSubsystem(SubsystemManager.WRIST_ROLL_KEY, s_WristRoll::setGoal, s_WristRoll::atGoal)
    );

    private final TrajectoryLibrary trajectoryLibrary = new TrajectoryLibrary(
        () -> new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(Constants.Swerve.swerveKinematics),
        Constants.Swerve.swerveKinematics,
        Constants.AutoConstants.kPXController,
        Constants.AutoConstants.kPYController,
        Constants.AutoConstants.kPThetaController,
        Constants.AutoConstants.kThetaControllerConstraints
    );

    /* auto stuff */
    public SendableChooser<Command> autoChooser;
    
    
        /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_VisionSubystem.setRobotPoseSupplier(s_Swerve::getPose);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> false //For the love of god do not change this
            )
        );

        GraphParser.getNodeByName("Stow").ifPresent(subsystemManager::setCurrentNode);

        // Configure the button bindings
        configureButtonBindings();

        configureAutoChooser();
    }
    
        /**
         * Use this method to define your button->command mappings. Buttons can be created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
            /* Driver Buttons */
            zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }

    private void configureAutoChooser() {
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("2 piece test auto", new TwoPiece(s_Swerve, trajectoryLibrary));
        autoChooser.addOption("Example trajectory", new exampleAuto(s_Swerve, trajectoryLibrary));

        SmartDashboard.putData(autoChooser);
    }

    public SubsystemManager getSubsystemManager() {
        return subsystemManager;
    }

    public TrajectoryLibrary getTrajectoryLibrary() {
        return trajectoryLibrary;
    }

    public AutoAlignController getAutoAlignController() {
        return autoAlignController;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
