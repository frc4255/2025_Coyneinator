package frc.robot.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ConstraintsTest {

    @Test
    void pivotStaysPutWhenElevatorExtendedBeyondThreshold() {
        double currentPivot = 0.2; // rad
        double desiredPivot = 1.0; // rad
        double elevatorPosition = 0.25; // metres, above retract threshold

        double clamped = Constraints.clampPivot(desiredPivot, currentPivot, elevatorPosition);
        assertEquals(currentPivot, clamped, 1e-9, "Pivot should hold when elevator is extended");
    }

    @Test
    void pivotCanNudgeWithinHoldWindow() {
        double currentPivot = 0.5;
        double desiredPivot = currentPivot + Math.toRadians(1.0); // within 2 degree window
        double elevatorPosition = 0.3;

        double clamped = Constraints.clampPivot(desiredPivot, currentPivot, elevatorPosition);
        assertEquals(desiredPivot, clamped, 1e-9, "Small pivot adjustment within hold window should be allowed");
    }

    @Test
    void elevatorClampedByPivotAngle() {
        double desiredElevator = 0.5; // metres
        double currentElevator = 0.0;
        double pivotRadians = Math.toRadians(5.0); // below low threshold

        double clamped = Constraints.clampElevator(desiredElevator, currentElevator, pivotRadians, false);
        assertEquals(0.15, clamped, 1e-9, "Elevator should be capped while pivot is low");
    }

    @Test
    void elevatorExpansionLimitedDuringClimbProfile() {
        double desiredElevator = 0.6;
        double currentElevator = 0.2;
        double pivotRadians = Math.toRadians(30.0);

        double clamped = Constraints.clampElevator(desiredElevator, currentElevator, pivotRadians, true);
        assertEquals(currentElevator + 0.02, clamped, 1e-9, "Climb profile should limit elevator rate while pivot is low");
    }

    @Test
    void pitchClampedByPivotAndReefClearance() {
        double desiredPitch = Math.toRadians(-80.0);
        double pivotRadians = Math.toRadians(10.0); // low pivot -> shallow pitch allowed
        double elevatorMeters = 0.2; // below reef clearance

        double clamped = Constraints.clampPitch(desiredPitch, pivotRadians, elevatorMeters, false);
        assertEquals(Math.toRadians(-15.0), clamped, 1e-9, "Pitch should respect the most conservative limit");
    }

    @Test
    void rollStaysWithinBounds() {
        double desiredRoll = Math.toRadians(400.0);
        double clamped = Constraints.clampRoll(desiredRoll);
        assertTrue(clamped <= Math.toRadians(200.0));

        desiredRoll = Math.toRadians(-400.0);
        clamped = Constraints.clampRoll(desiredRoll);
        assertTrue(clamped >= Math.toRadians(-200.0));
    }
}
