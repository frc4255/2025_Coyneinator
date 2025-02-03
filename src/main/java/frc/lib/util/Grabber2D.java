package frc.lib.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.*;

public class Grabber2D {
    private final Elevator elevator;
    private final Arm arm;
    private final Pivot pivot;

    private final double armLength = Constants.Grabber2D.armLength; 
    private final double minHeight = Constants.Grabber2D.minHeight; 
    private final double maxHeight = Constants.Grabber2D.maxHeight; 
    
    private final double wristLength = Constants.Grabber2D.wristLength;


    

    public Grabber2D(Elevator elevator, Arm arm, Pivot pivot) {
        this.elevator = elevator;
        this.arm = arm;
        this.pivot = pivot;
    }

    public void moveTo(double x, double y) {
        
        if (y < minHeight || y > maxHeight || Math.abs(x) > armLength) {
            throw new IllegalArgumentException("Target out of bounds");
        }
  
        double radians = Math.atan2(x, y);
        
        elevator.setPos(y);
        arm.setPos(radians);
    }

    public void moveToWithFixedAngle(double y, double angle) {

        if (y < minHeight || y > maxHeight) {
            throw new IllegalArgumentException("Target out of bounds");
        }

        double HeightOfArm = (armLength * Math.sin(angle)); 
        elevator.setPos(y - HeightOfArm);
        arm.setPos(angle);

        double valueOfX = (armLength * Math.cos(angle));
        
        SmartDashboard.putNumber("Distance of arm from elevator", valueOfX); 
    }

    public void CompBotMovement(double x, double y, double wristAngle) {

        double elevatorX = x - (wristLength * Math.cos(wristAngle));

        double elevatorY = y - (wristLength * Math.sin(wristAngle));

        double elevatorExtension = Math.sqrt(Math.pow(elevatorX, 2) + Math.pow(elevatorY, 2));

        double angleOfElevator = Math.asin(elevatorY / elevatorExtension);

        elevator.setPos(elevatorExtension);
        pivot.setPos(angleOfElevator);

        arm.setPos(wristAngle);

        
        

    }
}

