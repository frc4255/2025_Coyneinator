package frc.robot.superstructure;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.List;

import org.junit.jupiter.api.Test;

import frc.lib.util.graph.GraphParser;
import frc.lib.util.graph.Node;

class PlanPathingTest {
    private static final List<ManipulatorPlan> ALL_PLANS = List.of(
            PlanLibrary.groundIntakeAndHandoff(),
            PlanLibrary.groundIntakeHold(),
            PlanLibrary.stow(),
            PlanLibrary.scoreL1(),
            PlanLibrary.scoreL2(),
            PlanLibrary.scoreL3(),
            PlanLibrary.scoreL4(),
            PlanLibrary.algaeReefPickup(),
            PlanLibrary.algaeGroundIntake(),
            PlanLibrary.algaeProcessorScore(),
            PlanLibrary.algaeBargeScore(),
            PlanLibrary.climbReady(),
            PlanLibrary.climbFinish()
    );

    private static final List<String> COMMON_START_NODES = List.of(
            "Stow",
            "Reef Align",
            "Intake Intermediate",
            "Net Intermediate"
    );

    @Test
    void commonStartNodesReachFirstStepOfEachPlan() {
        for (ManipulatorPlan plan : ALL_PLANS) {
            String firstStepName = plan.steps().get(0).nodeName();
            Node firstNode = GraphParser.getNodeByName(firstStepName);
            assertNotNull(firstNode, () -> "Plan " + plan.name() + " references missing node " + firstStepName);

            for (String startName : COMMON_START_NODES) {
                Node start = GraphParser.getNodeByName(startName);
                assertNotNull(start, () -> "Missing common start node " + startName);
                // Skip trivial case where start and first step are identical.
                if (start == firstNode) {
                    continue;
                }
                var path = GraphParser.getFastestPath(start, firstNode);
                assertFalse(path.isEmpty(), () -> "No path from " + startName + " to first step " + firstStepName + " of plan " + plan.name());
            }
        }
    }

    @Test
    void consecutivePlanStepsAreReachable() {
        for (ManipulatorPlan plan : ALL_PLANS) {
            List<ManipulatorPlan.Step> steps = plan.steps();
            for (int i = 0; i < steps.size() - 1; i++) {
                String fromName = steps.get(i).nodeName();
                String toName = steps.get(i + 1).nodeName();
                Node from = GraphParser.getNodeByName(fromName);
                Node to = GraphParser.getNodeByName(toName);
                assertNotNull(from, () -> "Missing node " + fromName + " in plan " + plan.name());
                assertNotNull(to, () -> "Missing node " + toName + " in plan " + plan.name());
                var path = GraphParser.getFastestPath(from, to);
                assertFalse(path.isEmpty(), () -> "Plan " + plan.name() + " has no path from " + fromName + " to " + toName);
            }
        }
    }
}
