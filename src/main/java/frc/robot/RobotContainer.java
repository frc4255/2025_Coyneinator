package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.lib.sim.SwerveSim;
import frc.lib.sim.TeleopSwerveSim;
import frc.lib.util.subsystemController;
import frc.lib.util.OnTheFlyTrajectory;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.Camera;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.wrist.EndEffector;
import frc.robot.subsystems.wrist.WristManager;

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

    /* When viewed from behind the bot */ //OFFSETS NEED TO BE REDONE
    private final Camera RightCam = new Camera(new PhotonCamera("RightCam"), 
        new Transform3d(new Translation3d(0.258, -0.291, 0.2), //TODO re-do offsets
        new Rotation3d(0, -1.08, -0.523)));
        
    private final Camera LeftCam = new Camera(new PhotonCamera("LeftCam"), 
        new Transform3d(new Translation3d(0.258, 0.291, 0.2), //TODO re-do offsets
        new Rotation3d(0, -1.08, 0.523))); 
    //private final Camera LLCam = new Camera(new PhotonCamera("LLCam"), new Transform3d(new Translation3d(0.135, 0, 0.204), new Rotation3d(0, -1.04, 0)));
    /* Driver Buttons */

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton toggleRobotState = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton stow = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final POVButton scoreL1 = new POVButton(driver, 0);
    private final POVButton scoreL2 = new POVButton(driver, 90);
    private final POVButton scoreL3 = new POVButton(driver, 180);
    private final POVButton scoreL4 = new POVButton(driver, 270);

    private final JoystickButton scoreNet = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton intakeFromHP = new JoystickButton(driver, XboxController.Button.kX.value);

    private final JoystickButton autoPathPlanningInTeleop = new JoystickButton(driver, XboxController.Button.kA.value);
    private boolean autoAlign = false;
    
        /* Subsystems */
        private final VisionSubsystem s_VisionSubystem = new VisionSubsystem(
                new Camera[]{LeftCam, RightCam}/*new Camera[]{}/*new Camera[]{rightCam, leftCam}*/);
                
        private final Swerve s_Swerve = new Swerve(s_VisionSubystem);

        private final SwerveSim s_SwerveSim = new SwerveSim(s_VisionSubystem);
    
        private final StateManager s_RobotState = new StateManager();
    
        private final Elevator s_Elevator = new Elevator();
        private final Pivot s_Pivot = new Pivot();
        private final Arm s_Arm = new Arm();
        private final WristManager s_Wrist = new WristManager();
        private final EndEffector s_EndEffector = new EndEffector();
        
        private final subsystemController subsystemController = new subsystemController(s_Elevator, s_Wrist, s_Pivot);
    
        private final Stow s_Stow = new Stow(s_Elevator, s_Pivot, s_Wrist, subsystemController);

        /* auto stuff */
        public SendableChooser<Command> autoChooser;

        intake autoIntake = new intake(s_Elevator, s_Pivot, s_Wrist, s_EndEffector);
        driverManualScoring autoScoreL4 = new driverManualScoring(s_Elevator, s_Wrist, 
                StateManager.Positions.L4, subsystemController);
        Stow autoStow = new Stow(s_Elevator, s_Pivot, s_Wrist, subsystemController);

        OnTheFlyTrajectory OnTheFlyTrajectory = new OnTheFlyTrajectory(s_Swerve);
    
    
    
        /** The container for the robot. Contains subsystems, OI devices, and commands. */
        public RobotContainer() {
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> -driver.getRawAxis(rotationAxis), 
                    () -> false //For the love of god do not change this
                )
            );

            s_SwerveSim.setDefaultCommand(
                new TeleopSwerveSim(
                    s_SwerveSim, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> -driver.getRawAxis(rotationAxis), 
                    () -> false //For the love of god do not change this
                )
            );

            NamedCommands.registerCommand("Intake", autoIntake);
            NamedCommands.registerCommand("ScoreL4", autoScoreL4);
    
            // Configure the button bindings
            configureButtonBindings();

            
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
    
            toggleRobotState.onTrue(new InstantCommand(() -> s_RobotState.toggleRobotState()));
    
            stow.onTrue(s_Stow);
    
            scoreL1.onTrue(new driverManualScoring(s_Elevator, s_Wrist, StateManager.Positions.L1, subsystemController));
            scoreL2.onTrue(new driverManualScoring(s_Elevator, s_Wrist, StateManager.Positions.L2, subsystemController));
            scoreL3.onTrue(new driverManualScoring(s_Elevator, s_Wrist, StateManager.Positions.L3, subsystemController));
            scoreL4.onTrue(new driverManualScoring(s_Elevator, s_Wrist, StateManager.Positions.L4, subsystemController));
    
            scoreNet.onTrue(new driverManualScoring(s_Elevator, s_Wrist, StateManager.Positions.NET, subsystemController));
            intakeFromHP.onTrue(new driverManualScoring(s_Elevator, s_Wrist, StateManager.Positions.HP, subsystemController));
    
            autoPathPlanningInTeleop.onTrue(new InstantCommand(() -> {
                autoAlign = !autoAlign; 

                if (autoAlign) {
                    new scoringAutoAlign(s_Swerve, OnTheFlyTrajectory).schedule();
                } else {
                    CommandScheduler.getInstance().cancel(new scoringAutoAlign(s_Swerve, OnTheFlyTrajectory));
                }
            }));
        


    }

    private void configureAutoChooser() {
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("2 piece test auto", new TwoPiece(s_Swerve));

        SmartDashboard.putData(autoChooser);
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
