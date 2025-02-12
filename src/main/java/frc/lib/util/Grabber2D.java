package frc.lib.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

    private Alert alert = new Alert("Out of Bot's movement range", AlertType.kError);




    public Grabber2D(Elevator elevator, Arm arm, Pivot pivot, WristManager wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.pivot = pivot;
        this.wrist = wrist;
    }

    public void moveTo(double x, double y) {
        
        double radians = Math.atan2(x, y);
        
        elevator.setPos(y);
        arm.setPos(radians);
    }

    public void moveToWithFixedAngle(double y, double angle) {
    }

    public void BotShouldGoTo(double pivotAngle, double elevatorExtension, 
                            double wristPitchAngle, double wristRollAngle) {

        if (elevator.isElevatorPosePossible() || pivot.isPivotPosePossible() ||
            wrist.isRollPosePossible() || wrist.isPitchPosePossible()) {
            alert.set(false);
        } else {
            alert.set(true);
            throw new IllegalArgumentException("Out of Bot's range of movement");
        }

        pivot.setPos(pivotAngle);

        elevator.setPos(elevatorExtension);

        wrist.setPitch(wristPitchAngle);
        wrist.setRoll(wristRollAngle);
    }
}

