package frc.robot.superstructure;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * Represents a high level manipulator plan consisting of ordered steps.
 */
public final class ManipulatorPlan {
    private final String name;
    private final List<Step> steps;

    private ManipulatorPlan(String name, List<Step> steps) {
        this.name = name;
        this.steps = Collections.unmodifiableList(steps);
    }

    public String name() {
        return name;
    }

    public List<Step> steps() {
        return steps;
    }

    public static Builder builder(String name) {
        return new Builder(name);
    }

    public static final class Builder {
        private final String name;
        private final List<Step> steps = new ArrayList<>();

        private Builder(String name) {
            this.name = name;
        }

        public Builder addStep(Step step) {
            steps.add(Objects.requireNonNull(step));
            return this;
        }

        public Builder addStep(Step.Builder builder) {
            steps.add(builder.build());
            return this;
        }

        public ManipulatorPlan build() {
            return new ManipulatorPlan(name, List.copyOf(steps));
        }
    }

    @FunctionalInterface
    public interface StepAction {
        void run(ManipulatorContext context);
    }

    @FunctionalInterface
    public interface AdvanceCondition {
        boolean shouldAdvance(ManipulatorContext context);
    }

    /**
     * Single manipulator step.
     */
    public static final class Step {
        private final String nodeName;
        private final ManipulatorProfile profile;
        private final double[] tolerance;
        private final double minHoldTimeSeconds;
        private final boolean requiresConfirm;
        private final AdvanceCondition advanceCondition;
        private final StepAction onEnter;
        private final StepAction whileActive;
        private final StepAction onExit;
        private final double timeoutSeconds;

        private Step(
                String nodeName,
                ManipulatorProfile profile,
                double[] tolerance,
                double minHoldTimeSeconds,
                boolean requiresConfirm,
                AdvanceCondition advanceCondition,
                StepAction onEnter,
                StepAction whileActive,
                StepAction onExit,
                double timeoutSeconds
        ) {
            this.nodeName = nodeName;
            this.profile = profile;
            this.tolerance = tolerance;
            this.minHoldTimeSeconds = minHoldTimeSeconds;
            this.requiresConfirm = requiresConfirm;
            this.advanceCondition = advanceCondition;
            this.onEnter = onEnter;
            this.whileActive = whileActive;
            this.onExit = onExit;
            this.timeoutSeconds = timeoutSeconds;
        }

        public String nodeName() {
            return nodeName;
        }

        public ManipulatorProfile profile() {
            return profile;
        }

        public double[] tolerance() {
            return tolerance;
        }

        public double minHoldTimeSeconds() {
            return minHoldTimeSeconds;
        }

        public boolean requiresConfirm() {
            return requiresConfirm;
        }

        public AdvanceCondition advanceCondition() {
            return advanceCondition;
        }

        public StepAction onEnter() {
            return onEnter;
        }

        public StepAction whileActive() {
            return whileActive;
        }

        public StepAction onExit() {
            return onExit;
        }

        public double timeoutSeconds() {
            return timeoutSeconds;
        }

        public static Builder builder(String nodeName) {
            return new Builder(nodeName);
        }

        public static final class Builder {
            private final String nodeName;
            private ManipulatorProfile profile = ManipulatorProfile.TRANSIT;
            private double[] tolerance = profile.toleranceCopy();
            private double minHoldTimeSeconds = profile.defaultHoldTimeSeconds();
            private boolean requiresConfirm;
            private AdvanceCondition advanceCondition;
            private StepAction onEnter;
            private StepAction whileActive;
            private StepAction onExit;
            private double timeoutSeconds = -1.0;

            private Builder(String nodeName) {
                this.nodeName = nodeName;
            }

            public Builder profile(ManipulatorProfile profile) {
                this.profile = profile;
                if (tolerance == null) {
                    tolerance = profile.toleranceCopy();
                }
                return this;
            }

            public Builder tolerance(double[] tolerance) {
                this.tolerance = tolerance;
                return this;
            }

            public Builder minHoldTimeSeconds(double seconds) {
                this.minHoldTimeSeconds = seconds;
                return this;
            }

            public Builder requiresConfirm(boolean requiresConfirm) {
                this.requiresConfirm = requiresConfirm;
                return this;
            }

            public Builder advanceCondition(AdvanceCondition condition) {
                this.advanceCondition = condition;
                return this;
            }

            public Builder onEnter(StepAction action) {
                this.onEnter = action;
                return this;
            }

            public Builder whileActive(StepAction action) {
                this.whileActive = action;
                return this;
            }

            public Builder onExit(StepAction action) {
                this.onExit = action;
                return this;
            }

            public Builder timeoutSeconds(double seconds) {
                this.timeoutSeconds = seconds;
                return this;
            }

            public Step build() {
                return new Step(
                        nodeName,
                        profile,
                        tolerance != null ? tolerance : profile.toleranceCopy(),
                        minHoldTimeSeconds,
                        requiresConfirm,
                        advanceCondition,
                        onEnter,
                        whileActive,
                        onExit,
                        timeoutSeconds
                );
            }
        }
    }
}
