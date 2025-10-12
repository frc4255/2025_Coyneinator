package frc.robot.led;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.SubsystemManager;
import frc.robot.autoalign.ReefAutoAlign;
import frc.robot.subsystems.LEDHandler;
import frc.robot.superstructure.GamePieceState;
import frc.robot.superstructure.RobotSupervisor;

/**
 * Chooses LED patterns based on robot state and forwards them to the hardware handler.
 */
public final class LedCoordinator {
    private static final double FLASH_PERIOD = 0.35;

    private final LEDHandler ledHandler;
    private final RobotSupervisor supervisor;
    private final SubsystemManager subsystemManager;

    private Optional<ReefAutoAlign.AlignmentResult> alignmentResult = Optional.empty();
    private boolean autoAlignActive = false;

    private RobotSupervisor.Mode lastMode = RobotSupervisor.Mode.CORAL;
    private double celebrationEndTimestamp = -1.0;
    private boolean climbSettledDuringMode = false;
    private LedPattern lastAppliedPattern;

    public LedCoordinator(LEDHandler ledHandler, RobotSupervisor supervisor, SubsystemManager subsystemManager) {
        this.ledHandler = Objects.requireNonNull(ledHandler);
        this.supervisor = Objects.requireNonNull(supervisor);
        this.subsystemManager = Objects.requireNonNull(subsystemManager);
    }

    public void setAlignmentResult(ReefAutoAlign.AlignmentResult result) {
        alignmentResult = Optional.ofNullable(result);
    }

    public void clearAlignmentResult() {
        alignmentResult = Optional.empty();
    }

    public void setAutoAlignActive(boolean active) {
        autoAlignActive = active;
    }

    public void handleDisabledInit() {
        autoAlignActive = false;
        celebrationEndTimestamp = -1.0;
        climbSettledDuringMode = false;
        lastAppliedPattern = null;
        ledHandler.setDisabledAnimation(true);
    }

    public void update() {
        double now = Timer.getFPGATimestamp();
        RobotSupervisor.Mode currentMode = supervisor.getMode();
        if (currentMode == RobotSupervisor.Mode.CLIMB && subsystemManager.isTargetSettled()) {
            climbSettledDuringMode = true;
        }

        if (lastMode == RobotSupervisor.Mode.CLIMB && currentMode != RobotSupervisor.Mode.CLIMB) {
            if (climbSettledDuringMode) {
                celebrationEndTimestamp = now + Constants.LEDs.CELEBRATION_DURATION_SECONDS;
            }
            climbSettledDuringMode = false;
        }
        lastMode = currentMode;

        boolean disabled = DriverStation.isDisabled();
        ledHandler.setDisabledAnimation(disabled);
        if (disabled) {
            lastAppliedPattern = null;
            ledHandler.clearPattern();
            return;
        }

        PatternCandidate selection = selectPattern(now, currentMode);
        LedPattern selected = selection != null ? selection.pattern() : null;
        if (!Objects.equals(selected, lastAppliedPattern)) {
            ledHandler.setPattern(selected);
            lastAppliedPattern = selected;
        }

        String reason = selection != null ? selection.reason() : "None";
        Logger.recordOutput("LEDs/ActivePattern", selected != null ? selected.style().name() : "OFF");
        Logger.recordOutput("LEDs/ActiveReason", reason);
        Logger.recordOutput("LEDs/ActivePriority", selected != null ? selected.priority() : -1);
        Logger.recordOutput(
                "LEDs/ActivePrimaryRGB",
                selected != null ? selected.primary().toArray() : LedColor.BLACK.toArray()
        );
        Logger.recordOutput("LEDs/AutoAlignActive", autoAlignActive);
    }

    private PatternCandidate selectPattern(double now, RobotSupervisor.Mode mode) {
        List<PatternCandidate> candidates = new ArrayList<>();
        GamePieceState pieces = supervisor.getPieceState();
        boolean coralInManipulator = pieces.isCoralInWrist();
        boolean algaeInEndEffector = pieces.isAlgaeInEndEffector();
        boolean targetSettled = subsystemManager.isTargetSettled();
        boolean climbConstraintsActive = supervisor.lastRequestUsedClimbMode();
        boolean algaeMode = supervisor.isAlgaeMode();

        if (celebrationEndTimestamp > 0.0 && now < celebrationEndTimestamp) {
            candidates.add(
                    new PatternCandidate(
                            LedPattern.alternating(
                                    100,
                                    Constants.LEDs.DEEP_GREEN,
                                    Constants.LEDs.YELLOW,
                                    Constants.LEDs.CELEBRATION_PERIOD_SECONDS
                            ),
                            "ClimbCelebration"
                    )
            );
        }

        if (mode == RobotSupervisor.Mode.CLIMB || climbConstraintsActive) {
            if (targetSettled) {
                candidates.add(
                        new PatternCandidate(
                                LedPattern.solid(90, Constants.LEDs.DEEP_GREEN),
                                "ClimbStable"
                        )
                );
            } else {
                candidates.add(
                        new PatternCandidate(
                                LedPattern.flashing(95, Constants.LEDs.DEEP_GREEN, FLASH_PERIOD),
                                "ClimbAttempt"
                        )
                );
            }
        }

        if (algaeInEndEffector) {
            candidates.add(
                    new PatternCandidate(
                            LedPattern.flashing(80, Constants.LEDs.TEAL, FLASH_PERIOD),
                            "AlgaeInEndEffector"
                    )
            );
        } else if (mode == RobotSupervisor.Mode.ALGAE || algaeMode) {
            candidates.add(
                    new PatternCandidate(
                            LedPattern.solid(70, Constants.LEDs.TEAL),
                            "AlgaeMode"
                    )
            );
        }

        if (mode == RobotSupervisor.Mode.CORAL && coralInManipulator) {
            alignmentResult.ifPresent(result -> {
                boolean oddSector = (result.sectorIndex() % 2) == 1;
                LedColor branchColor = oddSector ? Constants.LEDs.YELLOW : Constants.LEDs.PURPLE;
                if (autoAlignActive) {
                    candidates.add(
                            new PatternCandidate(
                                    LedPattern.flashing(60, branchColor, FLASH_PERIOD),
                                    "AutoAlignBranch"
                            )
                    );
                } else {
                    candidates.add(
                            new PatternCandidate(
                                    LedPattern.solid(50, branchColor),
                                    "BranchSector"
                            )
                    );
                }
            });
        }

        if (mode == RobotSupervisor.Mode.CORAL) {
            candidates.add(
                    new PatternCandidate(
                            LedPattern.solid(40, Constants.LEDs.LIGHT_GREEN),
                            "CoralMode"
                    )
            );
        }

        if (candidates.isEmpty()) {
            return null;
        }

        return candidates.stream()
                .max(Comparator.comparingInt(candidate -> candidate.pattern().priority()))
                .orElse(null);
    }

    private record PatternCandidate(LedPattern pattern, String reason) {}
}
