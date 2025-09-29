package frc.robot.superstructure;

import frc.lib.util.graph.GraphParser;
import frc.robot.Constants;
import frc.robot.superstructure.ManipulatorPlan.Step;

/**
 * Houses reusable manipulator plans.
 */
public final class PlanLibrary {
    private PlanLibrary() {}

    public static ManipulatorPlan groundIntakeAndHandoff() {
        return ManipulatorPlan.builder("GroundIntakeHandoff")
                .addStep(
                        Step.builder("Intake Intermediate")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.05)
                                .build()
                )
                .addStep(
                        Step.builder("Coral Ground Pickup")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.05)
                                .onEnter(ctx -> ctx.endEffector().setDutyCycle(Constants.EndEffector.INTAKE_CORAL_VOLTS))
                                .whileActive(ctx -> {
                                    if (ctx.endEffector().getMotorCurrent() > Constants.EndEffector.CORAL_INTAKE_CURRENT_LIMIT) {
                                        ctx.endEffector().setDutyCycle(Constants.EndEffector.INTAKE_CORAL_HOLD_VOLTS);
                                    }
                                })
                                .advanceCondition(ctx -> {
                                    boolean detected = ctx.sensors().hasCoralIntakeSensor()
                                            ? ctx.sensors().coralDetectedInIntake()
                                            : ctx.endEffector().getMotorCurrent() > Constants.EndEffector.CORAL_INTAKE_CURRENT_TRIGGER;
                                    if (detected) {
                                        ctx.pieceState().setCoralInIntake(true);
                                    }
                                    return detected;
                                })
                                .onExit(ctx -> ctx.endEffector().setDutyCycle(Constants.EndEffector.INTAKE_CORAL_HOLD_VOLTS))
                                .build()
                )
                .addStep(
                        Step.builder("Intake Intermediate")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.05)
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.06)
                                .onEnter(ctx -> ctx.endEffector().setDutyCycle(Constants.EndEffector.HANDOFF_CORAL_VOLTS))
                                .advanceCondition(ctx -> {
                                    boolean detected = ctx.sensors().hasCoralWristSensor()
                                            ? ctx.sensors().coralDetectedAtWrist()
                                            : ctx.pieceState().isCoralInIntake();
                                    if (detected) {
                                        ctx.pieceState().setCoralInWrist(true);
                                        ctx.pieceState().setCoralInIntake(false);
                                    }
                                    return detected;
                                })
                                .onExit(ctx -> ctx.endEffector().setDutyCycle(Constants.EndEffector.HOLD_CORAL_VOLTS))
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.05)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan groundIntakeHold() {
        return ManipulatorPlan.builder("GroundIntakeHold")
                .addStep(
                        Step.builder("Intake Intermediate")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.05)
                                .build()
                )
                .addStep(
                        Step.builder("Coral Ground Pickup")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.05)
                                .onEnter(ctx -> ctx.endEffector().setDutyCycle(Constants.EndEffector.INTAKE_CORAL_VOLTS))
                                .advanceCondition(ctx -> {
                                    boolean detected = ctx.sensors().hasCoralIntakeSensor()
                                            ? ctx.sensors().coralDetectedInIntake()
                                            : ctx.endEffector().getMotorCurrent() > Constants.EndEffector.CORAL_INTAKE_CURRENT_TRIGGER;
                                    if (detected) {
                                        ctx.pieceState().setCoralInIntake(true);
                                    }
                                    return detected;
                                })
                                .onExit(ctx -> ctx.endEffector().setDutyCycle(Constants.EndEffector.HOLD_CORAL_VOLTS))
                                .build()
                )
                .addStep(
                        Step.builder("Intake Intermediate")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.05)
                                .onEnter(ctx -> ctx.endEffector().setDutyCycle(Constants.EndEffector.HOLD_CORAL_VOLTS))
                                .onExit(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan stow() {
        return ManipulatorPlan.builder("Stow")
                .addStep(
                        Step.builder("Stow")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.08)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan scoreL1() {
        return ManipulatorPlan.builder("ScoreL1")
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .build()
                )
                .addStep(
                        Step.builder("L1 Score")
                                .profile(ManipulatorProfile.SCORE)
                                .minHoldTimeSeconds(0.1)
                                .requiresConfirm(true)
                                .onExit(ctx -> {
                                    ctx.endEffector().setDutyCycle(Constants.EndEffector.OUTTAKE_CORAL_VOLTS);
                                    ctx.pieceState().setCoralInWrist(false);
                                })
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan scoreL2() {
        return ManipulatorPlan.builder("ScoreL2")
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .build()
                )
                .addStep(
                        Step.builder("L2 Align")
                                .profile(ManipulatorProfile.SCORE)
                                .requiresConfirm(true)
                                .minHoldTimeSeconds(0.1)
                                .onExit(ctx -> {
                                    ctx.endEffector().setDutyCycle(Constants.EndEffector.OUTTAKE_CORAL_VOLTS);
                                    ctx.pieceState().setCoralInWrist(false);
                                })
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan scoreL3() {
        return ManipulatorPlan.builder("ScoreL3")
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .build()
                )
                .addStep(
                        Step.builder("L3 Init")
                                .profile(ManipulatorProfile.TRANSIT)
                                .build()
                )
                .addStep(
                        Step.builder("L3 Dunk")
                                .profile(ManipulatorProfile.SCORE)
                                .requiresConfirm(true)
                                .minHoldTimeSeconds(0.12)
                                .onExit(ctx -> {
                                    ctx.endEffector().setDutyCycle(Constants.EndEffector.OUTTAKE_CORAL_FAST_VOLTS);
                                    ctx.pieceState().setCoralInWrist(false);
                                })
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan scoreL4() {
        return ManipulatorPlan.builder("ScoreL4")
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .build()
                )
                .addStep(
                        Step.builder("L4 Init")
                                .profile(ManipulatorProfile.TRANSIT)
                                .build()
                )
                .addStep(
                        Step.builder("L4 Dunk")
                                .profile(ManipulatorProfile.SCORE)
                                .requiresConfirm(true)
                                .minHoldTimeSeconds(0.12)
                                .onExit(ctx -> {
                                    ctx.endEffector().setDutyCycle(Constants.EndEffector.OUTTAKE_CORAL_FAST_VOLTS);
                                    ctx.pieceState().setCoralInWrist(false);
                                })
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan algaeReefPickup() {
        return ManipulatorPlan.builder("AlgaeReefPickup")
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .build()
                )
                .addStep(
                        Step.builder("L2 Algae Pickup")
                                .profile(ManipulatorProfile.SCORE)
                                .minHoldTimeSeconds(0.1)
                                .requiresConfirm(false)
                                .onEnter(ctx -> ctx.endEffector().setDutyCycle(Constants.EndEffector.INTAKE_ALGAE_VOLTS))
                                .advanceCondition(ctx -> ctx.sensors().hasAlgaeEndEffectorSensor()
                                        ? ctx.sensors().algaeDetectedAtEndEffector()
                                        : ctx.endEffector().getMotorCurrent() > Constants.EndEffector.ALGAE_INTAKE_CURRENT_TRIGGER)
                                .onExit(ctx -> {
                                    ctx.endEffector().setDutyCycle(Constants.EndEffector.HOLD_ALGAE_VOLTS);
                                    ctx.pieceState().setAlgaeInEndEffector(true);
                                })
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan algaeGroundIntake() {
        return ManipulatorPlan.builder("AlgaeGroundIntake")
                .addStep(
                        Step.builder("Algae Ground Pickup")
                                .profile(ManipulatorProfile.TRANSIT)
                                .onEnter(ctx -> ctx.endEffector().setDutyCycle(Constants.EndEffector.INTAKE_ALGAE_VOLTS))
                                .advanceCondition(ctx -> ctx.sensors().hasAlgaeIntakeSensor()
                                        ? ctx.sensors().algaeDetectedInIntake()
                                        : ctx.endEffector().getMotorCurrent() > Constants.EndEffector.ALGAE_INTAKE_CURRENT_TRIGGER)
                                .onExit(ctx -> {
                                    ctx.endEffector().setDutyCycle(Constants.EndEffector.HOLD_ALGAE_VOLTS);
                                    ctx.pieceState().setAlgaeInIntake(true);
                                })
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan algaeProcessorScore() {
        return ManipulatorPlan.builder("AlgaeProcessorScore")
                .addStep(
                        Step.builder("Processor Score")
                                .profile(ManipulatorProfile.SCORE)
                                .requiresConfirm(true)
                                .onExit(ctx -> {
                                    ctx.endEffector().setDutyCycle(Constants.EndEffector.OUTTAKE_ALGAE_VOLTS);
                                    ctx.pieceState().setAlgaeInEndEffector(false);
                                })
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan algaeBargeScore() {
        return ManipulatorPlan.builder("AlgaeBargeScore")
                .addStep(
                        Step.builder("Net Score")
                                .profile(ManipulatorProfile.SCORE)
                                .requiresConfirm(true)
                                .onExit(ctx -> {
                                    ctx.endEffector().setDutyCycle(Constants.EndEffector.OUTTAKE_ALGAE_VOLTS);
                                    ctx.pieceState().setAlgaeInEndEffector(false);
                                })
                                .build()
                )
                .addStep(
                        Step.builder("Reef Align")
                                .profile(ManipulatorProfile.TRANSIT)
                                .onEnter(ctx -> ctx.endEffector().stop())
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan climbReady() {
        return ManipulatorPlan.builder("ClimbReady")
                .addStep(
                        Step.builder("Climb")
                                .profile(ManipulatorProfile.CLIMB)
                                .minHoldTimeSeconds(0.1)
                                .build()
                )
                .build();
    }

    public static ManipulatorPlan climbFinish() {
        return ManipulatorPlan.builder("ClimbFinish")
                .addStep(
                        Step.builder("Climb End")
                                .profile(ManipulatorProfile.CLIMB)
                                .minHoldTimeSeconds(0.1)
                                .build()
                )
                .addStep(
                        Step.builder("Stow")
                                .profile(ManipulatorProfile.TRANSIT)
                                .minHoldTimeSeconds(0.1)
                                .build()
                )
                .build();
    }

    public static boolean nodeExists(String nodeName) {
        return GraphParser.getNodeByName(nodeName) != null;
    }
}
