package frc.lib.util;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.subsystems.*;

public class Grabber2D {
    private final Elevator elevator;
    private final Arm arm;

    private final double armLength; 
    private final double minHeight; 
    private final double maxHeight; 

    public Grabber2D(Elevator elevator, Arm arm, double armLength, double minHeight, double maxHeight) {
        this.elevator = elevator;
        this.arm = arm;
        this.armLength = armLength;
        this.minHeight = minHeight;
        this.maxHeight = maxHeight;
    }

    public void moveTo(double x, double y) {
        
        if (y < minHeight || y > maxHeight || Math.abs(x) > armLength) {
            throw new IllegalArgumentException("Target out of bounds");
        }
  
        double radians = Math.atan2(x, y);
        
        elevator.setPos(y);
        arm.setPos(radians);
    }
}

