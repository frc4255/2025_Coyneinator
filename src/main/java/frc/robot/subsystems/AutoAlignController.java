package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateManager;
import frc.robot.StateManager.RobotStateMachine;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AutoAlignController extends SubsystemBase {

    public static final double CORAL_OFFSET_METERS = Units.inchesToMeters(6.5);
    public static final double ROBOT_STAGING_RADIUS_METERS = Units.inchesToMeters(14.0);

    private final Joystick joystick;
    private final Supplier<RobotStateMachine> stateSupplier;

    private AutoAlignConfig config;
    private Consumer<Pose2d> poseListener = pose -> {};
    private Consumer<double[]> levelListener = level -> {};

    private Optional<Pose2d> lastRequestedPose = Optional.empty();
    private Optional<double[]> lastRequestedLevel = Optional.empty();

    public AutoAlignController(int port) {
        this(new Joystick(port));
    }

    public AutoAlignController(Joystick joystick) {
        this(joystick, AutoAlignConfig.empty(), StateManager::getCurrentState);
    }

    public AutoAlignController(int port, AutoAlignConfig config) {
        this(new Joystick(port), config, StateManager::getCurrentState);
    }

    public AutoAlignController(Joystick joystick, AutoAlignConfig config, Supplier<RobotStateMachine> stateSupplier) {
        this.joystick = joystick;
        this.stateSupplier = stateSupplier;
        setConfig(config);
    }

    public void setConfig(AutoAlignConfig config) {
        this.config = Objects.requireNonNullElse(config, AutoAlignConfig.empty());
    }

    public AutoAlignConfig getConfig() {
        return config;
    }

    public void setPoseListener(Consumer<Pose2d> poseListener) {
        this.poseListener = poseListener != null ? poseListener : pose -> {};
    }

    public void setLevelListener(Consumer<double[]> levelListener) {
        this.levelListener = levelListener != null ? levelListener : level -> {};
    }

    public Optional<Pose2d> getLastRequestedPose() {
        return lastRequestedPose;
    }

    public Optional<double[]> getLastRequestedLevel() {
        return lastRequestedLevel.map(level -> level.clone());
    }

    public void clearRequests() {
        lastRequestedPose = Optional.empty();
        lastRequestedLevel = Optional.empty();
    }

    @Override
    public void periodic() {
        RobotStateMachine state = stateSupplier.get();
        boolean coralMode = state == RobotStateMachine.Coral;
        boolean algaeMode = state == RobotStateMachine.Algae;

        if (coralMode || algaeMode) {
            for (AutoAlignConfig.ScoreButton button : config.scoringButtons()) {
                if (joystick.getRawButton(button.button())) {
                    Pose2d desiredPose = buildGoalPose(button, coralMode);
                    if (desiredPose != null) {
                        lastRequestedPose = Optional.of(desiredPose);
                        poseListener.accept(desiredPose);
                    }
                }
            }
        }

        config.levelButtons().forEach((button, target) -> {
            if (joystick.getRawButton(button)) {
                double[] coordinates = StateManager.getCoordinate(target);
                if (coordinates != null) {
                    double[] copy = coordinates.clone();
                    lastRequestedLevel = Optional.of(copy);
                    levelListener.accept(copy);
                }
            }
        });
    }

    private Pose2d buildGoalPose(AutoAlignConfig.ScoreButton button, boolean includeCoralOffset) {
        Alliance alliance = DriverStation.getAlliance().orElse(null);
        return config
            .resolveTag(button.tagSlot(), alliance)
            .map(tagPose -> {
                Translation2d translation = tagPose.getTranslation().plus(button.robotOffset());
                if (includeCoralOffset) {
                    translation = translation.plus(button.coralOffset());
                }
                return new Pose2d(translation, button.robotRotation());
            })
            .orElse(null);
    }
}
