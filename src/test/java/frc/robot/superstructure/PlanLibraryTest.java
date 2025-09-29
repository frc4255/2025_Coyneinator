package frc.robot.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

class PlanLibraryTest {

    @Test
    void scorePlansRequireConfirmationOnScoringStep() {
        assertStepRequiresConfirm(PlanLibrary.scoreL1(), "L1 Score");
        assertStepRequiresConfirm(PlanLibrary.scoreL2(), "L2 Align");
        assertStepRequiresConfirm(PlanLibrary.scoreL3(), "L3 Dunk");
        assertStepRequiresConfirm(PlanLibrary.scoreL4(), "L4 Dunk");
        assertStepRequiresConfirm(PlanLibrary.algaeProcessorScore(), "Processor Score");
        assertStepRequiresConfirm(PlanLibrary.algaeBargeScore(), "Net Score");
    }

    @Test
    void groundIntakePlanTransitionsThroughExpectedNodes() {
        ManipulatorPlan plan = PlanLibrary.groundIntakeAndHandoff();
        List<ManipulatorPlan.Step> steps = plan.steps();
        assertEquals("Intake Intermediate", steps.get(0).nodeName());
        assertEquals("Coral Ground Pickup", steps.get(1).nodeName());
        assertEquals("Intake Intermediate", steps.get(2).nodeName());
        assertEquals("Reef Align", steps.get(3).nodeName());
    }

    @Test
    void climbReadyPlanEntersClimbPose() {
        ManipulatorPlan plan = PlanLibrary.climbReady();
        List<ManipulatorPlan.Step> steps = plan.steps();
        assertEquals(1, steps.size());
        assertEquals("Climb", steps.get(0).nodeName());
        assertEquals(ManipulatorProfile.CLIMB, steps.get(0).profile());
    }

    private static void assertStepRequiresConfirm(ManipulatorPlan plan, String nodeName) {
        ManipulatorPlan.Step target = plan.steps().stream()
                .filter(step -> step.nodeName().equals(nodeName))
                .findFirst()
                .orElseThrow(() -> new AssertionError("Missing step " + nodeName + " in plan " + plan.name()));
        assertTrue(target.requiresConfirm(), () -> "Expected step " + nodeName + " to require confirmation in plan " + plan.name());
    }
}
