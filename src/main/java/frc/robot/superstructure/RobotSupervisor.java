package frc.robot.superstructure;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.graph.GraphParser;
import frc.robot.SubsystemManager;
import frc.robot.superstructure.ManipulatorPlan.Step;
import frc.robot.superstructure.ManipulatorPlan.StepAction;
import frc.robot.superstructure.ManipulatorPlan.AdvanceCondition;
import frc.robot.superstructure.ManipulatorProfile;
import frc.robot.subsystems.DifferentialWrist;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Pivot;

/**
 * Coordinates high-level intents and the manipulator plan execution.
 */
public final class RobotSupervisor {
    public enum HighLevelState {
        MANIPULATE,
        CLIMB,
        FAULT
    }

    public enum ScoreLevel {
        L1,
        L2,
        L3,
        L4
    }

    private final SubsystemManager manager;
    private final EndEffector endEffector;
    private final GroundIntake groundIntake;
    private final GamePieceState pieceState = new GamePieceState();
    private final PieceSensors sensors;
    private final ManipulatorContext context;

    private final Deque<ManipulatorPlan> planQueue = new ArrayDeque<>();

    private ManipulatorPlan currentPlan;
    private int currentStepIndex = -1;
    private Step activeStep;
    private double stepStartTime;
    private boolean waitingForConfirm;
    private boolean confirmQueued;
    private HighLevelState state = HighLevelState.MANIPULATE;
    private ScoreLevel activeScoreLevel;

    public RobotSupervisor(
            SubsystemManager manager,
            Pivot pivot,
            Elevator elevator,
            DifferentialWrist wrist,
            GroundIntake groundIntake,
            EndEffector endEffector,
            PieceSensors sensors
    ) {
        this.manager = Objects.requireNonNull(manager);
        this.endEffector = Objects.requireNonNull(endEffector);
        this.groundIntake = Objects.requireNonNull(groundIntake);
        this.sensors = Objects.requireNonNull(sensors);
        this.context = new ManipulatorContext(manager, pivot, elevator, wrist, groundIntake, endEffector, pieceState, sensors);
    }

    public void periodic() {
        pieceState.updateFromSensors(sensors);

        Logger.recordOutput("Supervisor/State", state.name());
        Logger.recordOutput("Supervisor/CurrentPlan", currentPlan != null ? currentPlan.name() : "none");
        Logger.recordOutput("Supervisor/CurrentStep", activeStep != null ? activeStep.nodeName() : "none");
        Logger.recordOutput("Supervisor/QueueSize", planQueue.size());
        Logger.recordOutput("Supervisor/WaitingForConfirm", waitingForConfirm);
        Logger.recordOutput("Supervisor/ConfirmQueued", confirmQueued);
        Logger.recordOutput("Supervisor/CoralIntake", pieceState.isCoralInIntake());
        Logger.recordOutput("Supervisor/CoralWrist", pieceState.isCoralInWrist());
        Logger.recordOutput("Supervisor/AlgaeIntake", pieceState.isAlgaeInIntake());
        Logger.recordOutput("Supervisor/AlgaeEffector", pieceState.isAlgaeInEndEffector());

        if (activeStep != null && activeStep.whileActive() != null) {
            activeStep.whileActive().run(context);
        }

        if (currentPlan == null) {
            pollQueue();
            return;
        }

        if (activeStep == null) {
            startNextStep();
            return;
        }

        double now = Timer.getFPGATimestamp();
        boolean timedOut = activeStep.timeoutSeconds() > 0.0 && (now - stepStartTime) >= activeStep.timeoutSeconds();
        if (timedOut && activeStep.requiresConfirm()) {
            confirmQueued = true;
        }

        boolean targetSettled = manager.isTargetSettled();
        boolean conditionMet = true;
        AdvanceCondition condition = activeStep.advanceCondition();
        if (condition != null) {
            conditionMet = condition.shouldAdvance(context);
        }

        if ((targetSettled && conditionMet) || timedOut) {
            if (activeStep.requiresConfirm()) {
                if (confirmQueued) {
                    waitingForConfirm = false;
                    confirmQueued = false;
                    runAction(activeStep.onExit());
                    pieceState.setCoralInIntake(false);
                    startNextStep();
                } else {
                    waitingForConfirm = true;
                }
            } else {
                runAction(activeStep.onExit());
                startNextStep();
            }
        }
    }

    public void queueConfirm() {
        confirmQueued = true;
        waitingForConfirm = false;
    }

    public boolean isWaitingForConfirm() {
        return waitingForConfirm;
    }

    public HighLevelState getState() {
        return state;
    }

    public void requestGroundIntakeHandoff() {
        if (state == HighLevelState.CLIMB) {
            return;
        }
        if (pieceState.isAlgaeInEndEffector()) {
            schedulePlan(PlanLibrary.groundIntakeHold(), false);
        } else {
            schedulePlan(PlanLibrary.groundIntakeAndHandoff(), false);
        }
        activeScoreLevel = null;
    }

    public void requestGroundIntakeHold() {
        if (state == HighLevelState.CLIMB) {
            return;
        }
        schedulePlan(PlanLibrary.groundIntakeHold(), false);
        activeScoreLevel = null;
    }

    public void requestStow(boolean clearQueue) {
        schedulePlan(PlanLibrary.stow(), clearQueue);
        activeScoreLevel = null;
    }

    public void requestScoreLevel(ScoreLevel level) {
        if (state == HighLevelState.CLIMB) {
            return;
        }

        if (activeStep != null && activeStep.requiresConfirm() && activeScoreLevel == level) {
            queueConfirm();
            return;
        }

        ManipulatorPlan plan = switch (level) {
            case L1 -> PlanLibrary.scoreL1();
            case L2 -> PlanLibrary.scoreL2();
            case L3 -> PlanLibrary.scoreL3();
            case L4 -> PlanLibrary.scoreL4();
        };

        activeScoreLevel = level;
        schedulePlan(plan, false);
    }

    public void requestAutoAlgae() {
        if (state == HighLevelState.CLIMB) {
            return;
        }

        schedulePlan(PlanLibrary.algaeReefPickup(), false);
        activeScoreLevel = null;
    }

    public void requestAlgaeGroundIntake() {
        if (state == HighLevelState.CLIMB) {
            return;
        }
        schedulePlan(PlanLibrary.algaeGroundIntake(), false);
        activeScoreLevel = null;
    }

    public void requestProcessorScore() {
        if (state == HighLevelState.CLIMB) {
            return;
        }
        if (waitingForConfirm && isCurrentPlan("AlgaeProcessorScore")) {
            queueConfirm();
            return;
        }
        schedulePlan(PlanLibrary.algaeProcessorScore(), false);
        activeScoreLevel = null;
    }

    public void requestBargeScore() {
        if (state == HighLevelState.CLIMB) {
            return;
        }
        if (waitingForConfirm && isCurrentPlan("AlgaeBargeScore")) {
            queueConfirm();
            return;
        }
        schedulePlan(PlanLibrary.algaeBargeScore(), false);
        activeScoreLevel = null;
    }

    public void toggleClimbMode() {
        if (state == HighLevelState.CLIMB) {
            state = HighLevelState.MANIPULATE;
            schedulePlan(PlanLibrary.stow(), true);
        } else {
            state = HighLevelState.CLIMB;
            schedulePlan(PlanLibrary.climbReady(), true);
        }
    }

    public void executeClimb() {
        if (state != HighLevelState.CLIMB) {
            return;
        }
        schedulePlan(PlanLibrary.climbFinish(), true);
    }

    public void clear() {
        planQueue.clear();
        currentPlan = null;
        currentStepIndex = -1;
        activeStep = null;
        waitingForConfirm = false;
        confirmQueued = false;
        manager.setInactive();
        endEffector.stop();
        pieceState.clearAll();
    }

    private void pollQueue() {
        if (currentPlan != null) {
            return;
        }
        ManipulatorPlan next = planQueue.pollFirst();
        if (next != null) {
            setPlan(next);
        }
    }

    private void schedulePlan(ManipulatorPlan plan, boolean clearQueue) {
        if (clearQueue) {
            planQueue.clear();
            currentPlan = null;
            activeStep = null;
            currentStepIndex = -1;
        }

        if (currentPlan == null) {
            setPlan(plan);
        } else {
            planQueue.addLast(plan);
        }
    }

    private void setPlan(ManipulatorPlan plan) {
        currentPlan = plan;
        currentStepIndex = -1;
        activeStep = null;
        waitingForConfirm = false;
        confirmQueued = false;
        Logger.recordOutput("Supervisor/PlanStarted", plan.name());
        startNextStep();
    }

    private boolean isCurrentPlan(String name) {
        return currentPlan != null && currentPlan.name().equals(name);
    }

    private void startNextStep() {
        if (currentPlan == null) {
            pollQueue();
            return;
        }

        currentStepIndex++;
        if (currentStepIndex >= currentPlan.steps().size()) {
            Logger.recordOutput("Supervisor/PlanCompleted", currentPlan.name());
            currentPlan = null;
            activeStep = null;
            currentStepIndex = -1;
            waitingForConfirm = false;
            confirmQueued = false;
            activeScoreLevel = null;
            pollQueue();
            return;
        }

        activeStep = currentPlan.steps().get(currentStepIndex);
        stepStartTime = Timer.getFPGATimestamp();
        waitingForConfirm = false;
        confirmQueued = false;

        StepAction onEnter = activeStep.onEnter();
        if (onEnter != null) {
            onEnter.run(context);
        }

        if (!PlanLibrary.nodeExists(activeStep.nodeName())) {
            Logger.recordOutput("Supervisor/UnknownNode", activeStep.nodeName());
        }

        manager.requestNode(
                GraphParser.getNodeByName(activeStep.nodeName()),
                activeStep.profile(),
                activeStep.tolerance(),
                activeStep.minHoldTimeSeconds()
        );
    }

    private void runAction(StepAction action) {
        if (action != null) {
            action.run(context);
        }
    }
}
