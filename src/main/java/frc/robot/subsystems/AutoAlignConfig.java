package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.StateManager;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Declarative configuration for the {@link AutoAlignController}. The config intentionally ships empty so teams can seed it with
 * the fields, buttons, and offsets they need without digging through control logic.
 */
public final class AutoAlignConfig {

    private final Map<Integer, AllianceTagPose> allianceTags;
    private final Map<Integer, ScoreButton> scoringButtons;
    private final Map<Integer, StateManager.Positions> levelButtons;

    private AutoAlignConfig(
        Map<Integer, AllianceTagPose> allianceTags,
        Map<Integer, ScoreButton> scoringButtons,
        Map<Integer, StateManager.Positions> levelButtons
    ) {
        this.allianceTags = Collections.unmodifiableMap(new HashMap<>(allianceTags));
        this.scoringButtons = Collections.unmodifiableMap(new HashMap<>(scoringButtons));
        this.levelButtons = Collections.unmodifiableMap(new HashMap<>(levelButtons));
    }

    public static AutoAlignConfig empty() {
        return new AutoAlignConfig(Map.of(), Map.of(), Map.of());
    }

    public Collection<ScoreButton> scoringButtons() {
        return scoringButtons.values();
    }

    public Map<Integer, StateManager.Positions> levelButtons() {
        return levelButtons;
    }

    public Optional<Pose2d> resolveTag(int tagSlot, Alliance alliance) {
        AllianceTagPose tagPose = allianceTags.get(tagSlot);
        if (tagPose == null) {
            return Optional.empty();
        }
        return Optional.of(tagPose.poseFor(alliance));
    }

    public Builder toBuilder() {
        Builder builder = new Builder();
        allianceTags.forEach((slot, pose) -> builder.addAllianceTag(slot, pose.bluePose(), pose.redPose()));
        scoringButtons.values().forEach(builder::addScoringButton);
        levelButtons.forEach(builder::addLevelButton);
        return builder;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static final class Builder {
        private final Map<Integer, AllianceTagPose> allianceTags = new HashMap<>();
        private final Map<Integer, ScoreButton> scoringButtons = new HashMap<>();
        private final Map<Integer, StateManager.Positions> levelButtons = new HashMap<>();

        public Builder addAllianceTag(int tagSlot, Pose2d bluePose, Pose2d redPose) {
            allianceTags.put(tagSlot, new AllianceTagPose(bluePose, redPose));
            return this;
        }

        public Builder addLevelButton(int button, StateManager.Positions level) {
            levelButtons.put(button, level);
            return this;
        }

        public Builder addScoringButton(ScoreButton buttonConfig) {
            scoringButtons.put(buttonConfig.button(), buttonConfig);
            return this;
        }

        public Builder addScoringButton(int button, ScoreButton buttonConfig) {
            scoringButtons.put(button, buttonConfig);
            return this;
        }

        public Builder addScoringButton(
            int button,
            int tagSlot,
            Translation2d robotOffset,
            Translation2d coralOffset,
            Rotation2d robotRotation
        ) {
            return addScoringButton(button, new ScoreButton(button, tagSlot, robotOffset, coralOffset, robotRotation));
        }

        public Builder removeScoringButton(int button) {
            scoringButtons.remove(button);
            return this;
        }

        public Builder removeLevelButton(int button) {
            levelButtons.remove(button);
            return this;
        }

        public Builder removeAllianceTag(int tagSlot) {
            allianceTags.remove(tagSlot);
            return this;
        }

        public AutoAlignConfig build() {
            return new AutoAlignConfig(allianceTags, scoringButtons, levelButtons);
        }
    }

    public record ScoreButton(
        int button,
        int tagSlot,
        Translation2d robotOffset,
        Translation2d coralOffset,
        Rotation2d robotRotation
    ) {}

    public record AllianceTagPose(Pose2d bluePose, Pose2d redPose) {
        public Pose2d poseFor(Alliance alliance) {
            if (alliance == Alliance.Red) {
                return redPose;
            }
            return bluePose;
        }
    }
}
