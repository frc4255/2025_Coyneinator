package frc.robot;

import java.security.Timestamp;
import java.util.Map;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
    private final JoystickButton swerveangletest = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton groundIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
        /* Subsystems */
        private final VisionSubsystem s_VisionSubystem = new VisionSubsystem(
                new Camera[]{LeftCam, RightCam}/*new Camera[]{}/*new Camera[]{rightCam, leftCam}*/);
                
        private final Swerve s_Swerve = new Swerve(s_VisionSubystem);
    
        private final Pivot s_Pivot = new Pivot();
        private final Elevator s_Elevator = new Elevator(s_Pivot::getPivotPosition);
        private final WristPitch s_WristPitch = new WristPitch();
        private final WristRoll s_WristRoll = new WristRoll();
        private final EndEffector s_EndEffector = new EndEffector();

        private final SubsystemManager manager = new SubsystemManager(s_Pivot, s_Elevator, s_WristPitch, s_WristRoll);
        
        /* auto stuff */
        public SendableChooser<Command> autoChooser;   
    
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

            // Configure the button bindings
            configureButtonBindings();

            configureAutoChooser();

            addTuningSliders();
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
            swerveangletest.onTrue(new test(s_Elevator));
            groundIntake.toggleOnTrue(new CoralGroundIntake(manager, s_EndEffector));

    }

    private void configureAutoChooser() {
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("2 piece test auto", new TwoPiece(s_Swerve));

        SmartDashboard.putData(autoChooser);
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
        return autoChooser.getSelected();
    }
}
