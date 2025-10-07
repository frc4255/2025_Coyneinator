package frc.robot;

import java.io.IOException;

import org.photonvision.PhotonCamera;

import choreo.auto.AutoFactory;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.Camera;
import frc.robot.superstructure.PieceSensors;
import frc.robot.superstructure.RobotSupervisor;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

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

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton stow = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton groundIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton algaeL2Pickup = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton algaeL3Pickup = new JoystickButton(driver, XboxController.Button.kRightStick.value);

    private final JoystickButton processorScore = new JoystickButton(driver, XboxController.Button.kX.value);

    private final JoystickButton runElevatorTesting = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton scoreBarge = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton algaeGroundIntake = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final CommandXboxController driverAgain = new CommandXboxController(0);
    private final Trigger extakeAlgae = new Trigger(driverAgain.leftTrigger(0.1));
    private final Trigger extakeCoral = new Trigger(driverAgain.rightTrigger(0.1));

    private final JoystickButton reefAlign = new JoystickButton(driver, XboxController.Button.kB.value);

    private final JoystickButton climb = new JoystickButton(driver, XboxController.Button.kStart.value);

    private final POVButton L1 = new POVButton(driver, 90);
    private final POVButton L2 = new POVButton(driver, 180);
    private final POVButton L3 = new POVButton(driver, 270);
    private final POVButton L4 = new POVButton(driver, 0);
    
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

            manager = new SubsystemManager(s_Pivot, s_Elevator, s_DifferentialWrist);

            pieceSensors = new PieceSensors();
            supervisor = new RobotSupervisor(manager, s_Pivot, s_Elevator, s_DifferentialWrist, s_GroundIntake, s_EndEffector, pieceSensors);

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
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */

        public void setManagerAsInactive () {
            supervisor.clear();
            manager.setInactive();
        }

        public void updateManager() {
            manager.update();
            supervisor.periodic();
        }
        
        private void configureButtonBindings() {
            zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

            stow.onTrue(new InstantCommand(() -> supervisor.requestStow(true)));
            groundIntake.onTrue(new InstantCommand(supervisor::requestGroundIntakeHandoff));
            runElevatorTesting.onTrue(new InstantCommand(supervisor::requestGroundIntakeHold));

            L1.onTrue(new InstantCommand(() -> supervisor.requestScoreLevel(RobotSupervisor.ScoreLevel.L1)));
            L2.onTrue(new InstantCommand(() -> supervisor.requestScoreLevel(RobotSupervisor.ScoreLevel.L2)));
            L3.onTrue(new InstantCommand(() -> supervisor.requestScoreLevel(RobotSupervisor.ScoreLevel.L3)));
            L4.onTrue(new InstantCommand(() -> supervisor.requestScoreLevel(RobotSupervisor.ScoreLevel.L4)));

            algaeL2Pickup.onTrue(new InstantCommand(supervisor::requestAlgaeGroundIntake));
            processorScore.onTrue(new InstantCommand(supervisor::requestProcessorScore));
            scoreBarge.onTrue(new InstantCommand(supervisor::requestBargeScore));
            reefAlign.onTrue(new InstantCommand(supervisor::requestAutoAlgae));

            climb.onTrue(new InstantCommand(supervisor::toggleClimbMode));
            algaeL3Pickup.onTrue(new InstantCommand(supervisor::executeClimb));

            extakeCoral.whileTrue(new ExtakeCoral(s_EndEffector));
            extakeAlgae.whileTrue(new ExtakeAlgae(s_EndEffector));

            zeroWristRoll.onTrue(new InstantCommand(s_DifferentialWrist::setHomed));
        }
    private void configureAutoChooser() {
        autochooser = new SendableChooser<>();
        autochooser.addOption("4 piece left", new OnePieceL1(s_Swerve, null, s_Pivot, s_Elevator, s_DifferentialWrist, s_EndEffector, manager));
        SmartDashboard.putData(autochooser);
    }
    
    private void addTuningSliders() {
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
