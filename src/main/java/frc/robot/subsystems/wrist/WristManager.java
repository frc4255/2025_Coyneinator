package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.wristdofcontrol.*;

public class WristManager extends SubsystemBase{
    private final PitchController pitchController;
    private final RollController rollController;

    private boolean isPitchPosePossible = true;
    private boolean isRollPosePossible = true;

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

    public double getPitch() {
        return pitchController.getCurrentAngle();
    }

    public double getRoll() {
        return rollController.getCurrentAngle();
    }

    public void stopPitchMotor(){
        pitchController.Stop();
    }

    public void stopRollMotor(){
        rollController.Stop();
    }

    public boolean isPitchPosePossible() {
        return isPitchPosePossible;
    }

    public boolean isRollPosePossible() {
        return isRollPosePossible;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (getPitch() > Constants.Wrist.PitchMaxLimit ||
            getPitch() < Constants.Wrist.PitchMinLimit) {

            isPitchPosePossible = false;
        } else {
            isPitchPosePossible = true;
        }

        if (getRoll() > Constants.Wrist.RollMaxLimit ||
            getPitch() < Constants.Wrist.RollMinLimit) {

            isRollPosePossible = false;
        } else {
            isRollPosePossible = true;
        }
    }
}
