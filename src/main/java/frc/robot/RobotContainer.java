package frc.robot;

import java.io.IOException;
import java.security.Timestamp;
import java.util.Map;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.lib.util.OnTheFlyTrajectory;
import frc.lib.util.graph.GraphParser;
import frc.robot.autos.*;
import frc.robot.autos.autocommands.testAuto;
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
  /*  private final Camera RightCam = new Camera(new PhotonCamera("RightCam"), 
        new Transform3d(new Translation3d(0.258, -0.291, 0.2), //TODO re-do offsets
        new Rotation3d(0, -1.08, -0.523)));
        
    private final Camera LeftCam = new Camera(new PhotonCamera("LeftCam"), 
        new Transform3d(new Translation3d(0.258, 0.291, 0.2), //TODO re-do offsets
        new Rotation3d(0, -1.08, 0.523))); 
    //private final Camera LLCam = new Camera(new PhotonCamera("LLCam"), new Transform3d(new Translation3d(0.135, 0, 0.204), new Rotation3d(0, -1.04, 0)));*/
    /* Driver Buttons */

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton stow = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton groundIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton extakeCoral = new JoystickButton(driver, XboxController.Button.kX.value);

    private final JoystickButton runElevatorTesting = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

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

        //private final OnTheFlyTrajectory onTheFlyTrajectory = new OnTheFlyTrajectory(s_Swerve);
        //private final AlignTool alignTool = new AlignTool();

        private final SubsystemManager manager = new SubsystemManager(s_Pivot, s_Elevator, s_WristPitch, s_WristRoll);
        
        /* auto stuff */
        public SendableChooser<Command> autoChooser;
    
        /** The container for the robot. Contains subsystems, OI devices, and commands. 
                 * @throws ParseException 
                 * @throws IOException 
                 * @throws FileVersionException */
                public RobotContainer() throws FileVersionException, IOException, ParseException {
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> driver.getRawAxis(rotationAxis), 
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

    }

    private void configureAutoChooser() throws FileVersionException, IOException, ParseException {

        PathPlannerPath path0 = PathPlannerPath.fromPathFile("1Pc");

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("nothing", null);
        autoChooser.addOption("4 piece left", new FourPieceNoHPL4(s_Swerve, s_Pivot, null, null, s_Elevator, s_WristPitch, s_WristRoll, s_EndEffector, manager));
        autoChooser.addOption("test", s_Swerve.followPathCommand(path0));
        // TODO: fix? V
        autoChooser.addOption("JustSwerveMobile", new testAuto(s_Swerve));

        
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
