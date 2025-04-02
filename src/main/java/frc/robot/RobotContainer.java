package frc.robot;

import java.io.IOException;

import org.photonvision.PhotonCamera;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.subsystems.Vision.Camera;
import frc.robot.subsystems.Vision.VisionSubsystem;


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
  /*  private final Camera RightCam = new Camera(new PhotonCamera("RightCam"), 
        new Transform3d(new Translation3d(0.258, -0.291, 0.2), //TODO re-do offsets
        new Rotation3d(0, -1.08, -0.523)));
        
    private final Camera LeftCam = new Camera(new PhotonCamera("LeftCam"), 
        new Transform3d(new Translation3d(0.258, 0.291, 0.2), //TODO re-do offsets
        new Rotation3d(0, -1.08, 0.523))); 
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
                new Camera[]{}/*new Camera[]{}/*new Camera[]{rightCam, leftCam}*/);
                
        private final Swerve s_Swerve = new Swerve(s_VisionSubystem);
    
        private final Pivot s_Pivot = new Pivot();
        private final Elevator s_Elevator = new Elevator(s_Pivot::getPivotPosition);
        private final WristPitch s_WristPitch = new WristPitch();
        private final WristRoll s_WristRoll = new WristRoll();
        private final EndEffector s_EndEffector = new EndEffector();
        private final Climber s_Climber = new Climber();
        //private final OnTheFlyTrajectory onTheFlyTrajectory = new OnTheFlyTrajectory(s_Swerve);
        //private final AlignTool alignTool = new AlignTool();

        private final SubsystemManager manager = new SubsystemManager(s_Pivot, s_Elevator, s_WristPitch, s_WristRoll);
        
        /* auto stuff */
        private SendableChooser<Command> autochooser;
        //private final AutoChooser autoChooser;

        private final AutoFactory autoFactory;
    
        /** The container for the robot. Contains subsystems, OI devices, and commands. 
                 * @throws ParseException 
                 * @throws IOException 
                 * @throws FileVersionException */
        public RobotContainer() {
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> driver.getRawAxis(rotationAxis), 
                    () -> false //For the love of god do not change this
                )
            );

            s_WristRoll.setDefaultCommand(new WristRollManual(s_WristRoll, () -> operator.getRawAxis(operatorHorizontalAxis)));
            // Configure the button bindings
            configureButtonBindings();

            addTuningSliders();

            autoFactory = new AutoFactory(
            s_Swerve::getPose, // A function that returns the current robot pose
            s_Swerve::setPose, // A function that resets the current robot pose to the provided Pose2d
            s_Swerve::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            s_Swerve // The drive subsystem
        );

        configureAutoChooser();

            /*
        autoChooser = new AutoChooser();

        // Add options to the chooser
        autoChooser.addCmd("Example Routine", () -> new OnePieceL1(s_Swerve, null, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll, s_EndEffector, manager, autoFactory));

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
            manager.setInactive();
        }

        public void updateManager() {
            manager.update();
        }
        
        private void configureButtonBindings() {
            /* Driver Buttons */
            zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
            groundIntake.onTrue(new CoralGroundIntake(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

            L1.onTrue(new L1Assist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
            L2.onTrue(new L2Assist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
            L3.onTrue(new L3Assist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
            L4.onTrue(new L4Assist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

            stow.onTrue(new Stow(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

            runElevatorTesting.onTrue(new InstantCommand(() -> s_Elevator.setGoal(0.5)));

            extakeCoral.whileTrue(new ExtakeCoral(s_EndEffector));

            scoreBarge.onTrue(new NetAssist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

            algaeL2Pickup.onTrue(new AlgaeL2Pickup(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));
            algaeL3Pickup.onTrue(new AlgaeL3Pickup(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

            algaeGroundIntake.onTrue(new AlgaeGroundIntake(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

            coralHPIntake.onTrue(new CoralHumanPlayerIntake(manager, s_EndEffector, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

            processorScore.onTrue(new ProcessorAssist(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

            extakeAlgae.whileTrue(new ExtakeAlgae(s_EndEffector));

            reefAlign.onTrue(new ReefAlign(manager, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll));

            climb.onTrue(new ClimbAssist(manager, s_Climber, s_Pivot));

            zeroWristRoll.onTrue(new InstantCommand(() -> s_WristRoll.setHomed()));

            manualWristRoll.whileTrue(new InstantCommand(() -> s_WristRoll.controlManually(operatorHorizontalAxis)));

            

    }
    private void configureAutoChooser() {
        autochooser = new SendableChooser<>();
        autochooser.addOption("4 piece left", new OnePieceL1(s_Swerve, null, s_Pivot, s_Elevator, s_WristPitch, s_WristRoll, s_EndEffector, manager));            
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
