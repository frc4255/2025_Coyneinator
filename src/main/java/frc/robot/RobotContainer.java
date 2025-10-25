package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.GroundIntakeIO;
import frc.robot.subsystems.GroundIntakeIOReal;
import frc.robot.subsystems.GroundIntakeIOSim;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveIO;
import frc.robot.subsystems.SwerveIOReal;
import frc.robot.subsystems.SwerveIOSim;
import frc.robot.visualization.GroundIntakeVisualizer;

public class RobotContainer {
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double PITCH_STEP_RADIANS = Units.degreesToRadians(5.0);

    private final CommandXboxController driver = new CommandXboxController(0);

    private final Swerve swerve;
    private final GroundIntake groundIntake;
    private final GroundIntakeGraphManager groundIntakeManager;
    private final GroundIntakeVisualizer groundIntakeVisualizer;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        SwerveIO swerveIO = RobotBase.isReal() ? new SwerveIOReal() : new SwerveIOSim();
        GroundIntakeIO groundIntakeIO = RobotBase.isReal() ? new GroundIntakeIOReal() : new GroundIntakeIOSim();

        swerve = new Swerve(swerveIO);
        groundIntake = new GroundIntake(groundIntakeIO);
        groundIntakeManager = new GroundIntakeGraphManager(groundIntake);
        groundIntakeVisualizer = new GroundIntakeVisualizer(groundIntake);

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> false
            )
        );

        configureBindings();
        configureAutonomous();
    }

    private void configureBindings() {
        Trigger intakeTrigger = driver.rightTrigger(TRIGGER_THRESHOLD);
        Trigger outtakeTrigger = driver.leftTrigger(TRIGGER_THRESHOLD);

        intakeTrigger.whileTrue(
            Commands.startEnd(
                groundIntake::setCoralIntake,
                groundIntake::stopRollers,
                groundIntake
            )
        );

        outtakeTrigger.whileTrue(
            Commands.startEnd(
                groundIntake::setHandoffSpeeds,
                groundIntake::stopRollers,
                groundIntake
            )
        );

        driver.a().onTrue(
            Commands.runOnce(
                () -> {
                    groundIntakeManager.setInactive();
                    groundIntake.setAsHomed();
                },
                groundIntake
            )
        );

        driver.y().onTrue(
            Commands.runOnce(
                () -> groundIntakeManager.requestNodeByName("Idle"),
                groundIntake
            )
        );

        driver.x().onTrue(
            Commands.runOnce(
                () -> groundIntakeManager.requestNodeByName("Ground Intake"),
                groundIntake
            )
        );

        driver.b().onTrue(
            Commands.runOnce(
                () -> groundIntakeManager.requestNodeByName("Handoff"),
                groundIntake
            )
        );

        driver.rightBumper().onTrue(
            Commands.runOnce(() -> nudgePitch(PITCH_STEP_RADIANS), groundIntake)
        );
        driver.leftBumper().onTrue(
            Commands.runOnce(() -> nudgePitch(-PITCH_STEP_RADIANS), groundIntake)
        );
    }

    private void configureAutonomous() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void nudgePitch(double deltaRadians) {
        groundIntakeManager.setInactive();
        double goal = groundIntake.getPitchGoalPosition() + deltaRadians;
        double clamped = Math.max(
            Constants.GroundIntake.MIN_PITCH_RADIANS,
            Math.min(Constants.GroundIntake.MAX_PITCH_RADIANS, goal)
        );
        groundIntake.setPitchGoal(clamped);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void onDisabled() {
        groundIntake.stop();
        groundIntake.stopRollers();
        groundIntakeManager.setInactive();
    }

    public void periodic() {
        groundIntakeManager.update();
        groundIntakeVisualizer.update();
    }

    public Swerve getSwerve() {
        return swerve;
    }

    public GroundIntake getGroundIntake() {
        return groundIntake;
    }
}
