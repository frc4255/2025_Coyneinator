package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.wristdofcontrol.*;

public class WristManager extends SubsystemBase{
    private final PitchController pitchController;
    private final RollController rollController;

    public WristManager(){
        pitchController = new PitchController(Constants.Wrist.PITCH_MOTOR_ID);
        rollController = new RollController(Constants.Wrist.ROLL_MOTOR_ID);
    }

    public void setPitch(double angle){
        pitchController.setPitchAngleRAD(angle);
    }

    public void setRoll(double angle){
        rollController.setRollAngleRAD(angle);
    }

    public void getPitch() {
        pitchController.getCurrentAngle();
    }

    public void getRoll() {
        rollController.getCurrentAngle();
    }

    public void stopPitch(){
        pitchController.Stop();
    }

    public void stopRoll(){
        rollController.Stop();
    }
}
