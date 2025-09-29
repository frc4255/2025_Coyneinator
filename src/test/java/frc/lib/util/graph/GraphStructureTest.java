package frc.lib.util.graph;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.superstructure.PlanLibrary;
import frc.robot.superstructure.ManipulatorPlan;

class GraphStructureTest {

    private static Set<String> planNodes;

    @BeforeAll
    static void gatherPlanNodes() {
        planNodes = new HashSet<>();
        addNodes(PlanLibrary.groundIntakeAndHandoff());
        addNodes(PlanLibrary.groundIntakeHold());
        addNodes(PlanLibrary.stow());
        addNodes(PlanLibrary.scoreL1());
        addNodes(PlanLibrary.scoreL2());
        addNodes(PlanLibrary.scoreL3());
        addNodes(PlanLibrary.scoreL4());
        addNodes(PlanLibrary.algaeReefPickup());
        addNodes(PlanLibrary.algaeGroundIntake());
        addNodes(PlanLibrary.algaeProcessorScore());
        addNodes(PlanLibrary.algaeBargeScore());
        addNodes(PlanLibrary.climbReady());
        addNodes(PlanLibrary.climbFinish());
    }

    private static void addNodes(ManipulatorPlan plan) {
        plan.steps().forEach(step -> planNodes.add(step.nodeName()));
    }

    @Test
    void everyPlanNodeExistsInGraph() {
        for (String nodeName : planNodes) {
            Node node = GraphParser.getNodeByName(nodeName);
            assertNotNull(node, () -> "Missing node in graph_data.json: " + nodeName);
        }
    }

    @Test
    void stowHasPathsToEveryPlanNode() {
        Node stow = GraphParser.getNodeByName("Stow");
        assertNotNull(stow, "Stow node must exist in the graph");

        for (String nodeName : planNodes) {
            Node target = GraphParser.getNodeByName(nodeName);
            assertNotNull(target, () -> "Missing node in graph_data.json: " + nodeName);
            if (target == stow) {
                continue;
            }
            List<Node> path = GraphParser.getFastestPath(stow, target);
            assertFalse(path.isEmpty(), () -> "No path from Stow to " + nodeName);
        }
    }
}
