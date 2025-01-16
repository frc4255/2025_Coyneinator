package frc.lib.util;

import java.lang.constant.Constable;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Grabber2D {
    private final Elevator elevator;
    private final Arm arm;

    private final double armLength = Constants.Grabber2D.armLength; 
    private final double minHeight = Constants.Grabber2D.minHeight; 
    private final double maxHeight = Constants.Grabber2D.maxHeight; 

    public Grabber2D(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
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

