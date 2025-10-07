package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.graph.GraphParser;
import frc.robot.autos.TrajectoryLibrary;
import frc.robot.autos.TwoPiece;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.AlgaeGroundIntake;
import frc.robot.commands.AlgaeL2Pickup;
import frc.robot.commands.AlgaeL3Pickup;
import frc.robot.commands.CoralGroundIntake;
import frc.robot.commands.CoralHumanPlayerIntake;
import frc.robot.commands.ExtakeAlgae;
import frc.robot.commands.ExtakeCoral;
import frc.robot.commands.L1Assist;
import frc.robot.commands.L2Assist;
import frc.robot.commands.L3Assist;
import frc.robot.commands.L4Assist;
import frc.robot.commands.NetAssist;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.Score;
import frc.robot.commands.Stow;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.test;
import frc.robot.subsystems.AutoAlignConfig;
import frc.robot.subsystems.AutoAlignController;
import frc.robot.subsystems.AutoAlignPresets;
import frc.robot.subsystems.CoordinatedSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorIOSim;
import frc.robot.subsystems.ElevatorIOTalonFX;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.PivotIOSim;
import frc.robot.subsystems.PivotIOTalonFX;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveIO;
import frc.robot.subsystems.SwerveIOSim;
import frc.robot.subsystems.SwerveIOTalonFX;
import frc.robot.subsystems.SwerveIOReplay;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOPhoton;
import frc.robot.subsystems.Vision.VisionIOPhoton.CameraConfig;
import frc.robot.subsystems.Vision.VisionIOSim;
import frc.robot.subsystems.Vision.VisionObservation;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.sim.Coyneinator3dVisualizer;
import frc.robot.subsystems.WristPitch;
import frc.robot.subsystems.WristPitchIOSim;
import frc.robot.subsystems.WristPitchIOTalonFX;
import frc.robot.subsystems.WristRoll;
import frc.robot.subsystems.WristRollIOSim;
import frc.robot.subsystems.WristRollIOTalonFX;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /* Driver Triggers */
    private final Trigger zeroGyro = driverController.y();
    private final Trigger toggleRobotState = driverController.rightBumper();
    private final Trigger climb = driverController.a();
    private final Trigger reefAlignDriver = driverController.b();
    private final Trigger runElevatorTesting = driverController.start();

    /* Operator Triggers */
    private final Trigger groundIntake = operatorController.a();
    private final Trigger coralHPIntake = operatorController.b();
    private final Trigger scoreBarge = operatorController.x();
    private final Trigger stow = operatorController.y();
    private final Trigger levelL1 = operatorController.povDown();
    private final Trigger levelL2 = operatorController.povLeft();
    private final Trigger levelL3 = operatorController.povUp();
    private final Trigger levelL4 = operatorController.povRight();
    private final Trigger algaeGroundIntake = operatorController.leftBumper();
    private final Trigger algaeL2Pickup = operatorController.rightBumper();
    private final Trigger algaeL3Pickup = operatorController.start();
    private final Trigger processorScore = operatorController.back();
    private final Trigger rightTrigger = operatorController.rightTrigger(0.3);
    private final Trigger leftTrigger = operatorController.leftTrigger(0.3);
    private final Trigger zeroWristRoll = operatorController.leftStick();
    private final Trigger manualWristRoll = operatorController.rightStick();

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

    private VisionIO visionIOBackend;
    private final Random visionSimRandom = new Random();

    /* Subsystems */
    private final StateManager stateManager = new StateManager();
    private final EndEffector s_EndEffector = new EndEffector();
    private final VisionSubsystem s_VisionSubsystem = new VisionSubsystem(createVisionIO());
    private final Swerve s_Swerve = new Swerve(createSwerveIO(), s_VisionSubsystem);
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
    private final Coyneinator3dVisualizer simVisualizer;

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
        VisionIO io;
        VisionIOMode mode = getRequestedVisionIOMode();
        switch (mode) {
            case PHOTON:
                io = new VisionIOPhoton(VISION_CAMERA_CONFIGS);
                break;
            case SIM:
                io = new VisionIOSim();
                break;
            case AUTO:
            default:
                io = RobotBase.isReal()
                    ? new VisionIOPhoton(VISION_CAMERA_CONFIGS)
                    : new VisionIOSim();
                break;
        }
        visionIOBackend = io;
        return io;
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
                System.out.println(
                    "Ignoring requested swerve IO mode '" + requestedMode + "' because the robot is running on real hardware."
                );
            }
            return new SwerveIOTalonFX();
        }

        return switch (requestedMode) {
            case REAL -> new SwerveIOTalonFX();
            case SIM -> new SwerveIOSim();
            case REPLAY -> {
                SwerveIO replay = attemptLoadReplay();
                if (replay != null) {
                    yield replay;
                }
                System.err.println("Falling back to SwerveIOSim after replay log load failed.");
                yield new SwerveIOSim();
            }
            case AUTO -> {
                SwerveIO replay = attemptLoadReplay();
                yield replay != null ? replay : new SwerveIOSim();
            }
        };
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

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_VisionSubsystem.setRobotPoseSupplier(s_Swerve::getPose);

        simVisualizer =
            RobotBase.isSimulation()
                ? new Coyneinator3dVisualizer(s_Swerve, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll)
                : null;

        if (visionIOBackend instanceof VisionIOSim simIO) {
            simIO.setObservationSupplier(this::generateSimVisionObservations);
        }
        s_VisionSubsystem.setKnownCameraNames(
            VISION_CAMERA_CONFIGS.stream().map(CameraConfig::name).collect(Collectors.toList())
        );

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> false
            )
        );

        GraphParser.getNodeByName("Stow").ifPresent(subsystemManager::setCurrentNode);

        configureButtonBindings();
        configureAutoChooser();
    }

    /**
     * Use this method to define your button->command mappings.
     */
    public void setManagerAsInactive() {
        subsystemManager.setInactive();
    }

    public void updateManager() {
        subsystemManager.update();
        if (simVisualizer != null) {
            simVisualizer.update();
        }
    }

    private void configureButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(s_Swerve::zeroHeading, s_Swerve));
        toggleRobotState.onTrue(new InstantCommand(stateManager::toggleRobotState));

        groundIntake.onTrue(new CoralGroundIntake(subsystemManager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
        coralHPIntake.onTrue(new CoralHumanPlayerIntake(subsystemManager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
        scoreBarge.onTrue(new NetAssist(subsystemManager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll, s_EndEffector));
        stow.onTrue(new Stow(subsystemManager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

        levelL1.onTrue(new L1Assist(subsystemManager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
        levelL2.onTrue(new L2Assist(subsystemManager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
        levelL3.onTrue(new L3Assist(subsystemManager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
        levelL4.onTrue(new L4Assist(subsystemManager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

        algaeGroundIntake.onTrue(new AlgaeGroundIntake(subsystemManager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
        algaeL2Pickup.onTrue(new AlgaeL2Pickup(subsystemManager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
        algaeL3Pickup.onTrue(new AlgaeL3Pickup(subsystemManager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

        runElevatorTesting.onTrue(new InstantCommand(() -> s_Elevator.setGoal(0.5), s_Elevator));

        rightTrigger.whileTrue(new ExtakeCoral(s_EndEffector));
        leftTrigger.whileTrue(new ExtakeAlgae(s_EndEffector));

        processorScore.onTrue(
            new Score(
                4,
                subsystemManager,
                s_Pivot,
                s_Elevator,
                s_WristPitch,
                s_WristRoll,
                s_Swerve,
                s_EndEffector,
                rightTrigger::getAsBoolean
            )
        );

        reefAlignDriver.onTrue(new ReefAlign(subsystemManager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
        climb.onTrue(new test(s_Swerve));

        zeroWristRoll.onTrue(new InstantCommand(s_WristRoll::setHomed, s_WristRoll));
        manualWristRoll
            .whileTrue(
                new RunCommand(
                    () -> s_WristRoll.controlManually(operatorController.getLeftX()),
                    s_WristRoll
                )
            )
            .onFalse(new InstantCommand(s_WristRoll::stopMotor, s_WristRoll));
    }

    private void configureAutoChooser() {
        autoChooser.setDefaultOption("2 piece test auto", new TwoPiece(s_Swerve, trajectoryLibrary));
        autoChooser.addOption("Example trajectory", new exampleAuto(s_Swerve, trajectoryLibrary));
        SmartDashboard.putData("Auto Mode", autoChooser);
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

    private List<VisionObservation> generateSimVisionObservations() {
        if (s_Swerve == null) {
            return List.of();
        }

        Pose2d robotPose = s_Swerve.getPose();
        double timestamp = Timer.getFPGATimestamp();
        List<VisionObservation> observations = new ArrayList<>(VISION_CAMERA_CONFIGS.size());

        for (CameraConfig config : VISION_CAMERA_CONFIGS) {
            Pose2d noisyPose = new Pose2d(
                robotPose.getX() + visionSimRandom.nextGaussian() * 0.03,
                robotPose.getY() + visionSimRandom.nextGaussian() * 0.03,
                robotPose.getRotation().plus(new Rotation2d(visionSimRandom.nextGaussian() * Math.toRadians(2.0)))
            );
            observations.add(new VisionObservation(config.name(), noisyPose, timestamp, 0.1));
        }

        return observations;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
