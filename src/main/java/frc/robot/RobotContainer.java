package frc.robot;

import org.photonvision.PhotonCamera;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.security.cert.CertPathBuilderException;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import frc.robot.FieldLayout;
import frc.robot.FieldLayout.AlgaeHeight;
import frc.robot.FieldLayout.Branch;
import frc.robot.FieldLayout.Face;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.Camera;
import frc.robot.superstructure.GamePieceState;
import frc.robot.superstructure.PieceSensors;
import frc.robot.superstructure.RobotSupervisor;
import frc.robot.superstructure.RobotSupervisor.Mode;
import frc.robot.superstructure.RobotSupervisor.ScoreLevel;
import frc.robot.visualization.SuperstructureVisualizer;
import frc.robot.autoalign.ReefAutoAlign;
import frc.robot.autoalign.ReefAutoAlign.AlignmentResult;

import java.util.Set;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double GROUND_INTAKE_VOLTS = 6.0;
    private static final double AUTO_ALIGN_DISTANCE_EPS_METERS = Units.inchesToMeters(1.5);
    private static final double AUTO_ALIGN_ANGLE_EPS_RAD = Units.degreesToRadians(4.0);
    private static final double AUTO_ALIGN_LINEAR_SPEED_EPS = 0.12; // m/s
    private static final double AUTO_ALIGN_ANGULAR_SPEED_EPS = Units.degreesToRadians(6.0);

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);

    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Operator Controls */

    private final int operatorHorizontalAxis = XboxController.Axis.kLeftX.value;

    /* When viewed from behind the bot */ //OFFSETS NEED TO BE REDONE
    private final Camera leftFrontCam = new Camera(new PhotonCamera("Left_Forward"), 
        new Transform3d(new Translation3d(0.206, 0.265, 0.208), //TODO re-do offsets
        new Rotation3d(0, -1.08, 0.524)));
        
    private final Camera rightFrontCam = new Camera(new PhotonCamera("Right_Forward"), 
        new Transform3d(new Translation3d(0.206, -0.265, 0.208), //TODO re-do offsets
        new Rotation3d(0, -1.08, -0.524)));

    private final Camera rightRearCam = new Camera(new PhotonCamera("Right_Rear"), 
        new Transform3d(new Translation3d(-0.374, -0.262, 0.195), //TODO re-do offsets
        new Rotation3d(0, 0, -2.313)));

    private final Camera leftRearCam = new Camera(new PhotonCamera("Left_Rear"), 
        new Transform3d(new Translation3d(-0.374, 0.262, 0.195), //TODO re-do offsets
        new Rotation3d(0, 0, 2.313)));
    //private final Camera LLCam = new Camera(new PhotonCamera("LLCam"), new Transform3d(new Translation3d(0.135, 0, 0.204), new Rotation3d(0, -1.04, 0)));*/
    /* Operator Buttons */

    private final JoystickButton manualWristRoll = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton zeroWristRoll = new JoystickButton(operator, XboxController.Button.kBack.value);

    private final JoystickButton coralHPIntake = new JoystickButton(operator, XboxController.Button.kB.value);

    
        /* Subsystems */
        private final VisionSubsystem s_VisionSubystem = new VisionSubsystem(
            new Camera[]{rightFrontCam, leftFrontCam, rightRearCam, leftRearCam});

        private final Swerve s_Swerve;
        private final Pivot s_Pivot;
        private final Elevator s_Elevator;
        private final DifferentialWrist s_DifferentialWrist;
        private final GroundIntake s_GroundIntake;
        private final EndEffector s_EndEffector;
        private final Climber s_Climber;
        //private final OnTheFlyTrajectory onTheFlyTrajectory = new OnTheFlyTrajectory(s_Swerve);
        //private final AlignTool alignTool = new AlignTool();

        private final SubsystemManager manager;
        private final PieceSensors pieceSensors;
        private final RobotSupervisor supervisor;
        private final ReefAutoAlign reefAutoAlign = new ReefAutoAlign();
        private final SuperstructureVisualizer superstructureVisualizer;
        
        /* auto stuff */
        private SendableChooser<Command> autochooser;
        //private final AutoChooser autoChooser;

        private final AutoFactory autoFactory;
    
        /** The container for the robot. Contains subsystems, OI devices, and commands. */
        public RobotContainer() {
            SwerveIO swerveIO;
            PivotIO pivotIO;
            ElevatorIO elevatorIO;
            DifferentialWristIO differentialWristIO;
            GroundIntakeIO groundIntakeIO;
            EndEffectorIO endEffectorIO;
            ClimberIO climberIO;

            if (RobotBase.isReal()) {
                swerveIO = new SwerveIOReal();
                pivotIO = new PivotIOReal();
                elevatorIO = new ElevatorIOReal();
                differentialWristIO = new DifferentialWristIOReal();
                groundIntakeIO = new GroundIntakeIOReal();
                endEffectorIO = new EndEffectorIOReal();
                climberIO = new ClimberIOReal();
            } else {
                swerveIO = new SwerveIOSim();
                pivotIO = new PivotIOSim();
                elevatorIO = new ElevatorIOSim();
                differentialWristIO = new DifferentialWristIOSim();
                groundIntakeIO = new GroundIntakeIOSim();
                endEffectorIO = new EndEffectorIOSim();
                climberIO = new ClimberIOSim();
            }

            s_Swerve = new Swerve(swerveIO, s_VisionSubystem);
            s_Pivot = new Pivot(pivotIO);
            s_Elevator = new Elevator(elevatorIO, s_Pivot::getPivotPosition);
            s_DifferentialWrist = new DifferentialWrist(differentialWristIO);
            s_GroundIntake = new GroundIntake(groundIntakeIO);
            s_EndEffector = new EndEffector(endEffectorIO);
            s_Climber = new Climber(climberIO);

            manager = new SubsystemManager(s_Pivot, s_Elevator, s_DifferentialWrist, s_GroundIntake);

            pieceSensors = new PieceSensors();
            supervisor = new RobotSupervisor(manager, s_GroundIntake, s_EndEffector, pieceSensors);
            superstructureVisualizer = new SuperstructureVisualizer(
                s_Pivot,
                s_Elevator,
                s_DifferentialWrist,
                s_GroundIntake,
                SuperstructureVisualizer.Config.fromConstants()
            );

            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve,
                    () -> -driver.getRawAxis(translationAxis),
                    () -> -driver.getRawAxis(strafeAxis),
                    () -> driver.getRawAxis(rotationAxis),
                    () -> false //For the love of god do not change this
                )
            );

            configureButtonBindings();
            addTuningSliders();

            autoFactory = new AutoFactory(
                s_Swerve::getPose,
                s_Swerve::setPose,
                s_Swerve::followTrajectory,
                true,
                s_Swerve
            );

            configureAutoChooser();

            /*
        autoChooser = new AutoChooser();

        // Add options to the chooser
        autoChooser.addCmd("Example Routine", () -> new OnePieceL1(s_Swerve, null, s_Pivot, s_Elevator, s_DifferentialWrist, s_EndEffector, manager, autoFactory));

        // Put the auto chooser on the dashboard
        SmartDashboard.putData(autoChooser);

        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());*/
        }
    
        /**
         * Use this method to define your button->command mappings. Buttons can be created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses
         * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
         * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */

        public void setManagerAsInactive () {
            supervisor.clearAutomation();
            manager.setInactive();
        }

        public void updateManager() {
            manager.update();
            supervisor.periodic();
            logAutoAlignTarget();
        }

        public void updateVisualizer() {
            superstructureVisualizer.update();
        }
        
        private void configureButtonBindings() {
            driver.back().onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading(), s_Swerve));
            
            driver.a().onTrue(Commands.runOnce(() -> supervisor.goToTransit("Idle")));
            // Intaking

            driver.rightBumper().toggleOnTrue(
                Commands.either(
                    coralIntakeWithAlgae(),
                    coralIntakeToHandoff(),
                    () -> supervisor.isAlgaeMode()
                ).handleInterrupt((interruptGroundIntake())
                )
            );

            driver.leftBumper().toggleOnTrue(
                Commands.either(
                    algaeGroundIntake(),
                    coralIntakeToHold(),
                    () -> driver.rightTrigger(0.1).getAsBoolean()
                ).handleInterrupt(() -> {
                    s_EndEffector.stop();
                    supervisor.goToTransit("Idle");
                })
            );

            // Scoring
            //Top left trigger - L4 and Barge
          /*  driver.povUp().whileTrue(
                Commands.either(Barge, L4, coralHPIntake)
            );*/
        }
    private void configureAutoChooser() {
        autochooser = new SendableChooser<>();
        autochooser.addOption("4 piece left", new OnePieceL1(s_Swerve, null, s_Pivot, s_Elevator, s_DifferentialWrist, s_EndEffector, manager));
        SmartDashboard.putData(autochooser);
    }
    
    private void addTuningSliders() {
    }

    private void logAutoAlignTarget() {
        Pose2d currentPose = s_Swerve.getPose();
        boolean isRedAlliance = DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red)
                .orElse(false);
        reefAutoAlign.calculate(currentPose, isRedAlliance);
    }

    private Command autoAlignUntilStable(Pose2d targetPose) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    var result = reefAutoAlign.calculate(s_Swerve.getPose(), isRedAlliance());
                    ChassisSpeeds speeds = result.robotRelativeSpeeds();
                    s_Swerve.drive(
                            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                            speeds.omegaRadiansPerSecond,
                            false,
                            false
                    );
                },
                interrupted -> s_Swerve.drive(new Translation2d(), 0.0, false, true),
                () -> isPoseStable(targetPose),
                s_Swerve
        );
    }

    private boolean isPoseStable(Pose2d targetPose) {
        Pose2d current = s_Swerve.getPose();
        double posError = current.getTranslation().getDistance(targetPose.getTranslation());
        double angleError = Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());

        ChassisSpeeds speeds = s_Swerve.getChassisSpeeds();
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);

        return posError <= AUTO_ALIGN_DISTANCE_EPS_METERS
                && angleError <= AUTO_ALIGN_ANGLE_EPS_RAD
                && linearSpeed <= AUTO_ALIGN_LINEAR_SPEED_EPS
                && angularSpeed <= AUTO_ALIGN_ANGULAR_SPEED_EPS;
    }

    private Runnable interruptGroundIntake() {
        return () -> {
            s_EndEffector.stop();
            s_GroundIntake.stop();

            if (supervisor.isAlgaeMode()) {
                supervisor.goToTransit("Holding Algae");
            } else {
                supervisor.goToTransit("Idle");
            }
        };
    }

    private class lazyWrapper {
        AlignmentResult res;

        public void setRes(AlignmentResult res) {
            this.res = res;
        }

        public AlignmentResult getRes() {
            return res;
        }
    }

    private Command autoAlignToReefCommand() {

        lazyWrapper res = new lazyWrapper();

        return Commands.run(
                () -> {
                    Pose2d currentPose = s_Swerve.getPose();
                    boolean isRedAlliance = DriverStation.getAlliance()
                            .map(alliance -> alliance == DriverStation.Alliance.Red)
                            .orElse(false);
                    res.setRes(reefAutoAlign.calculate(currentPose, isRedAlliance));
                    ChassisSpeeds speeds = res.getRes().robotRelativeSpeeds();
                    s_Swerve.drive(
                            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                            speeds.omegaRadiansPerSecond,
                            false,
                            false
                    );
                }
        ).finallyDo(() -> s_Swerve.drive(new Translation2d(), 0.0, false, true)).until(() -> {
            return (s_Swerve.getPose().getTranslation().getDistance(res.getRes().targetPose().getTranslation()) < 0.1) && (Math.abs(s_Swerve.getPose().getRotation().getRadians() - res.getRes().targetPose().getRotation().getRadians()) < 0.1);
        });
    }

    private Command algaeGroundIntake() {
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() -> supervisor.goToTransit("Ground Intake Algae")),
                Commands.runOnce(() -> s_EndEffector.setAlgaeIntakeSpeed())
            ),
            Commands.waitUntil(pieceSensors::algaeDetectedAtEndEffector),
            Commands.parallel(
                Commands.runOnce(() -> supervisor.recordAlgaeAcquisition()),
                rumbleCommand(0.5),
                Commands.runOnce(() -> supervisor.goToTransit("Holding Algae")),
                Commands.runOnce(() -> s_EndEffector.stop())
            )
        );
    }
    private Command coralIntakeWithAlgae() {
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() -> supervisor.goToTransit("Intake while Holding Algae")),
                Commands.runOnce(() -> s_GroundIntake.setCoralIntake())
                // LEDs
            ),
            Commands.waitUntil(pieceSensors::coralDetectedInIntake),
            Commands.parallel(
                    rumbleCommand(0.5),
                    Commands.runOnce(() -> supervisor.goToTransit("Holding Algae")),
                    Commands.runOnce(() -> s_GroundIntake.stopRollers())
                    // LEDs
            )
        );
    }

    private Command coralIntakeToHold() {
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() -> supervisor.goToTransit("Ground Intake")),
                Commands.runOnce(() -> s_GroundIntake.setCoralIntake())
        // LEDs
        ),
        Commands.waitUntil(pieceSensors::coralDetectedInIntake),
        Commands.parallel(
                rumbleCommand(0.5),
                Commands.runOnce(() -> supervisor.goToTransit("Idle")),
                Commands.runOnce(() -> s_GroundIntake.stopRollers())
        // LEDs
        )
        );
    }

    private Command HandoffCoralToWrist() {
        return Commands.sequence(
            Commands.runOnce(() -> supervisor.goToTransit("Handoff")),
            Commands.waitUntil(supervisor::isTargetSettled),
            Commands.parallel(
                    Commands.runOnce(() -> s_EndEffector.setCoralHandoffSpeed()),
                    Commands.runOnce(() -> s_GroundIntake.setHandoffSpeeds())),
            Commands.waitUntil(pieceSensors::coralDetectedAtWrist),
            Commands.parallel(
                Commands.runOnce(() -> s_EndEffector.stop()),
                Commands.runOnce(() -> s_GroundIntake.stopRollers())
            ),
            Commands.runOnce(() -> supervisor.goToTransit("Waiting to Score"))
        );
    }
    private Command coralIntakeToHandoff() {
        return Commands.sequence(
                Commands.parallel(
                        Commands.runOnce(() -> supervisor.goToTransit("Ground Intake")),
                        Commands.runOnce(() -> s_GroundIntake.setCoralIntake())
                // LEDs
                ),
                Commands.waitUntil(pieceSensors::coralDetectedInIntake).withTimeout(10),
                Commands.parallel(
                        rumbleCommand(0.5),
                        Commands.runOnce(() -> supervisor.goToTransit("Handoff")),
                        Commands.runOnce(() -> s_GroundIntake.stopRollers())
                // LEDs
                ),
                Commands.waitUntil(supervisor::isTargetSettled),
                Commands.parallel(
                        Commands.runOnce(() -> s_EndEffector.setCoralHandoffSpeed()),
                        Commands.runOnce(() -> s_GroundIntake.setHandoffSpeeds())),
                Commands.waitUntil(pieceSensors::coralDetectedAtWrist),
                Commands.parallel(
                    Commands.runOnce(() -> s_EndEffector.stop()),
                    Commands.runOnce(() -> s_GroundIntake.stopRollers())
                ),
                Commands.runOnce(() -> supervisor.goToTransit("Waiting to Score"))
        );
    }
    /*
    private Command coralIntakeToHandoff() {
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() -> supervisor.goToTransit("Intake while Holding Algae")),
                Commands.runOnce(() -> s_GroundIntake.setCoralIntake())
                //LEDs
            ),
            Commands.waitUntil(pieceSensors::coralDetectedInIntake),
            Commands.either(
                Commands.parallel(
                    rumbleCommand(0.5),
                    Commands.runOnce(() -> supervisor.goToTransit("Holding Algae")),
                    Commands.runOnce(() -> s_GroundIntake.stopRollers())
                    //LEDs
                ),
                () -> {
                    Commands.parallel(
                        rumbleCommand(0.5),
                        Commands.runOnce(() -> supervisor.goToTransit("Handoff")),
                        Commands.runOnce(() -> s_GroundIntake.stopRollers(), s_Swerve)
                        //LEDs
                    ),
                    Commands.waitUntil(supervisor::isTargetSettled),
                    //LEDs
                    Commands.parallel(
                        Commands.runOnce(() -> s_EndEffector.setCoralHandoffSpeed()),
                        Commands.runOnce(() -> s_GroundIntake.setHandoffSpeeds())  
                    ),
                    Commands.waitUntil(pieceSensors::coralDetectedAtWrist),
                    Commands.runOnce(() -> supervisor.goToTransit("Waiting to Score"))
                },
            () -> supervisor.getPieceState().isAlgaeInEndEffector())
            
        );
    }*/

    public Command rumbleCommand(double durationSeconds) {
		return Commands.sequence(
						Commands.runOnce(() -> {
							setRumble(true);
						}),
						Commands.waitSeconds(durationSeconds),
						Commands.runOnce(() -> {
							setRumble(false);
						}))
				.handleInterrupt(() -> {
					setRumble(false);
				});
	}

    public void setRumble(boolean on) {
		driver.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}
    private Command povAutomation(POVButton button, ScoreLevel coralLevel, Function<BooleanSupplier, Command> algaeFactory) {
        BooleanSupplier holdSupplier = button::getAsBoolean;
        return Commands.defer(
                () -> {
                    if (supervisor.getMode() == Mode.ALGAE) {
                        return algaeFactory.apply(holdSupplier);
                    }
                    return coralScoreWhileHeld(button, coralLevel);
                },
                Set.of(s_Swerve)
        );
    }

    private Command coralScoreWhileHeld(Trigger button, ScoreLevel level) {
        BooleanSupplier holdSupplier = button::getAsBoolean;
        return Commands.defer(
                () -> {
                    boolean isRed = isRedAlliance();
                    Branch branch = FieldLayout.getClosestBranch(s_Swerve.getPose(), isRed);
                    boolean useFront = shouldUseFront(branch, isRed);
                    Pose2d targetPose = FieldLayout.getRobotApproachPose(
                            branch,
                            isRed,
                            reefAutoAlign.getStandoffMeters());
                    ScoreLevel algaeLevel = FieldLayout.getAlgaeHeightForFace(branch.getKey().face());
                    Command align = autoAlignUntilStable(targetPose);
                    Command prepare = new InstantCommand();
                    String scoreNode = scoreNodeName(level, useFront);

                    GamePieceState pieceState = supervisor.getPieceState();
                    if (pieceState.isCoralInWrist()) {
                        if (!pieceState.isCoralInIntake()) {
                            return new InstantCommand();
                        } else {
                            prepare = ensureCoralReady();
                        }
                    }
                    Command sequence = Commands.sequence(
                            Commands.parallel(
                                align,
                                prepare
                            ),
                            Commands.runOnce(() -> supervisor.goToTransit("Waiting to Score")),
                            Commands.waitUntil(supervisor::isTargetSettled),
                            Commands.runOnce(() -> supervisor.goToScorePose(scoreNode)),
                            Commands.waitUntil(supervisor::isTargetSettled),
                            Commands.runOnce(
                                    () -> supervisor.runEndEffector(Constants.EndEffector.OUTTAKE_CORAL_VOLTS)),
                            Commands.waitSeconds(0.35), //TODO: Idk how I feel about this
                            Commands.runOnce(() -> {
                                supervisor.stopEndEffector();
                                supervisor.markCoralInWrist(false);
                                supervisor.markCoralInIntake(false);
                            }),
                            Commands.either(
                                    algaeReefFollowCommand(algaeLevel, branch, useFront),
                                    Commands.runOnce(supervisor::requestIdle),
                                    () -> {
                                        return supervisor.isAlgaeMode() || driver.rightTrigger(0.1).getAsBoolean();
                                    }),
                            Commands.waitUntil(() -> !holdSupplier.getAsBoolean()));
                    return sequence;
                },
                Set.of(s_Swerve, s_GroundIntake));
    }

    private Command algaeReefIntakeCommand(BooleanSupplier holdSupplier) {
        return Commands.defer(
                () -> {
                    Branch branch = FieldLayout.getClosestBranch(s_Swerve.getPose(), isRedAlliance());
                    boolean useFront = shouldUseFront(branch, isRedAlliance());
                    return algaeReefFollowCommand(ScoreLevel.L3, branch, useFront)
                            .andThen(Commands.waitUntil(() -> !holdSupplier.getAsBoolean()));
                },
                Set.of(s_Swerve));
    }

    private Command algaeBargeScoreCommand() {
        return Commands.defer(
                () -> Commands.sequence(
                        Commands.runOnce(() -> {
                            supervisor.setAlgaeMode(true);
                            supervisor.goToScorePose("Score Barge");
                        }),
                        Commands.waitUntil(supervisor::isTargetSettled),
                        Commands.runOnce(() -> supervisor.runEndEffector(Constants.EndEffector.OUTTAKE_ALGAE_VOLTS)),
                        Commands.waitSeconds(0.45),
                        Commands.runOnce(() -> {
                            supervisor.stopEndEffector();
                            supervisor.markAlgaeInEndEffector(false);
                            supervisor.markAlgaeInIntake(false);
                            supervisor.requestIdle();
                            supervisor.setAlgaeMode(false);
                        })
                ),
                Set.of(s_Swerve)
        );
    }

    private Command algaeProcessorScoreCommand() {
        return Commands.defer(
                () -> Commands.sequence(
                        Commands.runOnce(() -> {
                            supervisor.setAlgaeMode(true);
                            supervisor.goToScorePose("Score Processor");
                        }),
                        Commands.waitUntil(supervisor::isTargetSettled),
                        Commands.runOnce(() -> supervisor.runEndEffector(Constants.EndEffector.OUTTAKE_ALGAE_VOLTS)),
                        Commands.waitSeconds(0.45),
                        Commands.runOnce(() -> {
                            supervisor.stopEndEffector();
                            supervisor.markAlgaeInEndEffector(false);
                            supervisor.markAlgaeInIntake(false);
                            supervisor.requestIdle();
                            supervisor.setAlgaeMode(false);
                        })
                ),
                Set.of(s_Swerve)
        );
    }

    private Command algaeGroundIntakeCommand() {
        return Commands.defer(
                () -> Commands.sequence(
                        Commands.runOnce(() -> {
                            supervisor.setAlgaeMode(true);
                            supervisor.goToTransit("Ground Intake Algae");
                        }),
                        Commands.waitUntil(supervisor::isTargetSettled),
                        Commands.startEnd(
                                () -> {
                                    supervisor.runGroundIntakeRoller(GROUND_INTAKE_VOLTS);
                                    supervisor.runEndEffector(Constants.EndEffector.INTAKE_ALGAE_VOLTS);
                                },
                                () -> {
                                    supervisor.stopGroundIntake();
                                    supervisor.stopEndEffector();
                                },
                                s_GroundIntake
                        ).withTimeout(0.6),
                        Commands.runOnce(() -> {
                            supervisor.stopGroundIntake();
                            supervisor.stopEndEffector();
                            supervisor.markAlgaeInIntake(true);
                        }),
                        Commands.runOnce(() -> supervisor.goToTransit("Holding Algae")),
                        Commands.waitUntil(supervisor::isTargetSettled)
                ),
                Set.of(s_Swerve, s_GroundIntake)
        );
    }

    private Command coralIntakeHoldCommand() {
        return Commands.startEnd(
                () -> {
                    supervisor.setAlgaeMode(false);
                    supervisor.goToTransit("Ground Intake");
                    supervisor.runGroundIntakeRoller(GROUND_INTAKE_VOLTS);
                    supervisor.runEndEffector(Constants.EndEffector.INTAKE_CORAL_VOLTS);
                },
                () -> {
                    supervisor.stopGroundIntake();
                    supervisor.runEndEffector(Constants.EndEffector.INTAKE_CORAL_HOLD_VOLTS);
                    supervisor.markCoralInIntake(true);
                },
                s_GroundIntake
        );
    }

    private Command humanPlayerIntakeCommand() {
        return Commands.defer(
                () -> Commands.sequence(
                        Commands.runOnce(() -> supervisor.goToTransit("HP Intake")),
                        Commands.waitUntil(supervisor::isTargetSettled),
                        Commands.runOnce(() -> supervisor.runEndEffector(Constants.EndEffector.INTAKE_CORAL_VOLTS)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> {
                            supervisor.runEndEffector(Constants.EndEffector.HOLD_CORAL_VOLTS);
                            supervisor.markCoralInWrist(true);
                            supervisor.recordCoralAcquisition();
                            supervisor.markCoralInIntake(false);
                        })
                ),
                Set.of(s_Swerve)
        );
    }

    private Command climbInitCommand() {
        return Commands.defer(
                () -> Commands.sequence(
                        Commands.runOnce(() -> supervisor.goToClimbPose("Climb Init")),
                        Commands.waitUntil(supervisor::isTargetSettled)
                ),
                Set.of(s_Swerve)
        );
    }

    private Command climbFinishCommand() {
        return Commands.defer(
                () -> Commands.sequence(
                        Commands.runOnce(() -> supervisor.goToClimbPose("Climb Final")),
                        Commands.waitUntil(supervisor::isTargetSettled),
                        Commands.runOnce(() -> supervisor.requestIdle())
                ),
                Set.of(s_Swerve)
        );
    }

    private Command ensureCoralReady() {
        return Commands.either(
                HandoffCoralToWrist(),
                Commands.none(),
                () -> supervisor.getPieceState().isCoralInIntake() && !supervisor.getPieceState().isCoralInWrist()
        );
    }

    private Command algaeReefFollowCommand(ScoreLevel level, Branch branch, boolean useFront) {
        return Commands.sequence(
                Commands.runOnce(() -> supervisor.setAlgaeMode(true)),
                Commands.runOnce(() -> supervisor.goToTransit(algaeReefNodeName(level, useFront))),
                Commands.waitUntil(supervisor::isTargetSettled),
                Commands.runOnce(() -> supervisor.runEndEffector(Constants.EndEffector.INTAKE_ALGAE_VOLTS)),
                Commands.waitSeconds(0.6),
                Commands.runOnce(() -> {
                    supervisor.stopEndEffector();
                    supervisor.markAlgaeInEndEffector(true);
                    supervisor.recordAlgaeAcquisition();
                }),
                Commands.runOnce(() -> supervisor.goToTransit("Holding Algae")),
                Commands.waitUntil(supervisor::isTargetSettled)
        );
    }
/*
    private BooleanSupplier algaeFollowRequestedSupplier() {
        return () -> driverAgain.getRightTriggerAxis() > TRIGGER_THRESHOLD
                || supervisor.getPieceState().isAlgaeInEndEffector()
                || supervisor.getPieceState().isAlgaeInIntake();
    }*/

    private String scoreNodeName(ScoreLevel level, boolean front) {
        return switch (level) {
            case L1 -> "Score L1";
            case L2 -> front ? "Score L2 Front" : "Score L2 Back";
            case L3 -> front ? "Score L3 Front" : "Score L3 Back";
            case L4 -> front ? "Score L4 Front" : "Score L4 Back";
        };
    }

    private String algaeReefNodeName(ScoreLevel level, boolean front) {
        return switch (level) {
            case L4 -> front ? "Algae L3 Front" : "Algae L3 Back";
            case L3 -> front ? "Algae L2 Front" : "Algae L2 Back";
            case L2, L1 -> front ? "Algae L2 Front" : "Algae L2 Back";
        };
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().map(alliance -> alliance == Alliance.Red).orElse(false);
    }

    private boolean shouldUseFront(Branch branch, boolean isRedAlliance) {
        Rotation2d branchFacing = FieldLayout.getBranchFaceRotation(branch, isRedAlliance);
        Rotation2d robotHeading = s_Swerve.getPose().getRotation();
        double dot = Math.cos(robotHeading.minus(branchFacing).getRadians());
        return dot >= 0;
    }

    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autochooser.getSelected();
    }
}










