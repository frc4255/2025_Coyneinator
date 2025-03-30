package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDHandler extends SubsystemBase {

    private final CANdle candle;
    private final CANdleConfiguration config;

    private final int LEDCount;


    public LEDHandler(int deviceID, int LEDCount) {

        this.LEDCount = LEDCount;

        candle = new CANdle(deviceID);
        config = new CANdleConfiguration();

        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.GRB;
        config.brightnessScalar = 0.5;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(config, 100);
    }


    public void setStaticColorFullStrip(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    public void setStaticColor(int r, int g, int b, int startIndex, int count) {
        candle.setLEDs(r, g, b, 0, startIndex, count);
    }

    public void applyAnimation(Animation animation) {
        candle.animate(animation);
    }

    public void startRainbowAnimation(double brightness, double speed, int numLeds) {
        Animation rainbowAnimation = new RainbowAnimation(brightness, speed, numLeds);
        applyAnimation(rainbowAnimation);
    }

    public void startLarsonAnimation(int r, int g, int b, double speed, int numLeds, LarsonAnimation.BounceMode mode, int size) {
        Animation larsonAnimation = new LarsonAnimation(r, g, b, 0, speed, numLeds, mode, size);
        applyAnimation(larsonAnimation);
    }

    public void startStrobeAnimation(int r, int g, int b, double speed, int numLeds) {
        Animation strobeAnimation = new StrobeAnimation(r, g, b, 0, speed, numLeds);
        applyAnimation(strobeAnimation);
    }

    public void startColorFlowAnimation(int r, int g, int b, double speed, int numLeds) {
        Animation ColorFlowAnimation = new StrobeAnimation(r, g, b, 0, speed, numLeds);
        applyAnimation(ColorFlowAnimation);
    }

    public void chooseAnimation(String requestedAnimation, int r, int g, int b, double speed) {

        switch(requestedAnimation) {
            case "Rainbow":
                applyAnimation(new RainbowAnimation(config.brightnessScalar, speed, LEDCount));
                break;
            case "Larson":
                applyAnimation(new LarsonAnimation(r, g, b, 0, speed, LEDCount, LarsonAnimation.BounceMode.Center, LEDCount / 3)); //TODO completley random might look back so fix
                break;
            case "Strobe":
                applyAnimation(new StrobeAnimation(r, g, b, 0, speed, LEDCount));
                break;
            case "ColorFlow":
                applyAnimation(new StrobeAnimation(r, g, b, 0, speed, LEDCount));
                break;
            default:
                break;
        }

    }

}
