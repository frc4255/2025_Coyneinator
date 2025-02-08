package frc.lib.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.wrist.WristManager;

public class Grabber2D {
    private final Elevator elevator;
    private final Arm arm;
    private final Pivot pivot;
    private final WristManager wrist;

    private final double armLength = Constants.Grabber2D.armLength; 
    private final double elevatorMinHeight = Constants.Grabber2D.ElevatorminHeight; 
    private final double elevatorMaxHeight = Constants.Grabber2D.ElevatormaxHeight; 
    
    private final double wristLength = Constants.Grabber2D.wristLength;


    public Grabber2D(Elevator elevator, Arm arm, Pivot pivot, WristManager wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.pivot = pivot;
        this.wrist = wrist;
    }

    public void moveTo(double x, double y) {
        
        if (y < elevatorMinHeight || y > elevatorMaxHeight || Math.abs(x) > armLength) {
            throw new IllegalArgumentException("Target out of bounds");
        }
  
        double radians = Math.atan2(x, y);
        
        elevator.setPos(y);
        arm.setPos(radians);
    }

    public void moveToWithFixedAngle(double y, double angle) {

        if (y < elevatorMinHeight || y > elevatorMaxHeight) {
            throw new IllegalArgumentException("Target out of bounds");
        }

        double HeightOfArm = (armLength * Math.sin(angle)); 
        elevator.setPos(y - HeightOfArm);
        arm.setPos(angle);

        double valueOfX = (armLength * Math.cos(angle));
        
        SmartDashboard.putNumber("Distance of arm from elevator", valueOfX); 
    }

    public void BotShouldGoTo(double pivotAngle, double elevatorExtension, 
                            double wristPitchAngle, double wristRollAngle) {

        if (false) {
            throw new IllegalArgumentException("Out of Bot's range of movement");
        } //TODO finish adding constraints here.

        pivot.setPos(pivotAngle);

        elevator.setPos(elevatorExtension);

        wrist.setPitch(wristPitchAngle);
        wrist.setRoll(wristRollAngle);
    }
}

