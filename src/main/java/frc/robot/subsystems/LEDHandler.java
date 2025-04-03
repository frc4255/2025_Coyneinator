package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDRequest;
import frc.lib.util.LEDRequest.LEDAnimations;
import frc.robot.Constants;
import frc.robot.Constants.LEDs;

public class LEDHandler extends SubsystemBase {

    private final CANdle candle;
    private final CANdleConfiguration config;

    private final int LEDCount;

    private LEDRequest currentRequest = null;
    private LEDRequest defaultRequest;

    public LEDHandler(int deviceID, int LEDCount) {

        this.LEDCount = LEDCount;

        candle = new CANdle(deviceID);
        config = new CANdleConfiguration();

        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.5;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(config, 100);

        defaultRequest = new LEDRequest(LEDAnimations.RAINBOW, LEDCount, LEDCount, deviceID, LEDCount, null)
    }

    public void set(LEDRequest request) {

        if (request.priority < )
        
        if (request.hasAnimation()) {
            candle.animate(getAnimation(request));
        } else {
            candle.setLEDs(request.r, request.g, request.b);
        }
    }

    private Animation getAnimation(LEDRequest request) {
        switch (request.animation) {
            case STROBE:
                return new StrobeAnimation(
                    request.r,
                    request.g,
                    request.b,
                    0,
                    request.speed,
                    LEDCount
                );
            case LARSON:
                return new LarsonAnimation(
                    request.r,
                    request.g,
                    request.b,
                    0,
                    request.speed,
                    LEDCount,
                    BounceMode.Back,
                    3
                );
            case RAINBOW:
                return new RainbowAnimation(1, request.speed, LEDCount);
            default:
                return null;
        }
    }

}
