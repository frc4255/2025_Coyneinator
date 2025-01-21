package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.LEDconstants;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class LEDSystem extends SubsystemBase {
    private CANdle LEDs;
    private final int LEDS_PER_ANIMATION = 300;
    private XboxController joystick;
    private int CANDLE_channel = 0;
    private boolean CANDLE_clearAllAnims = false;
    private boolean CANDLE_animDirection = false;
    private boolean CANDLE_setAnim = false;

    private BooleanSupplier algaeSupplier;
    private BooleanSupplier coralSupplier;
    private BooleanSupplier alignedSupplier;

    // Game piece colors
    public static final Color algae = new Color(131, 227, 237);
    public static final Color coral = new Color(255, 255, 255);
    public static final Color nothing = new Color(0, 0, 0);

    // Indicator colors
    public static final Color white = new Color(255, 230, 220);
    public static final Color black = new Color(0, 0, 0);
    public static final Color green = new Color(56, 209, 0);
    public static final Color blue = new Color(8, 32, 255);
    public static final Color red = new Color(255, 0, 0);

    private Animation CANDLE_toAnimate = null;

    public class LEDSegment {
        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;
        private final CANdle LEDs;

        public LEDSegment(CANdle leds, int startIndex, int segmentSize, int animationSlot) {
            this.LEDs = leds;
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            clearAnimation();
            LEDs.setLEDs(color.RED, color.GREEN, color.BLUE, 0, startIndex, segmentSize);
        }

        public void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        public void clearAnimation() {
            LEDs.clearAnimation(animationSlot);
        }

        public void disableLEDs() {
            setColor(black);
        }
    }

    // Create instances of LEDSegment
    public LEDSegment BatteryIndicator = new LEDSegment(LEDs, 0, 2, 0);
    public LEDSegment MainStrip = new LEDSegment(LEDs, 2, LEDconstants.mainStripSegmentSize, 2);

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        Empty
    }
    private AnimationTypes CANDLE_currentAnimation;
    private AnimationTypes CANDLE_previousAnimation;

    public LEDSystem(BooleanSupplier algaeSupplier, BooleanSupplier coralSupplier, BooleanSupplier alignedSupplier,
            int startIndex, int segmentSize, int animationSlot) {
        this.algaeSupplier = algaeSupplier;
        this.coralSupplier = coralSupplier;
        this.alignedSupplier = alignedSupplier;
        LEDs = new CANdle(Constants.LEDconstants.CANDLE_ID);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        LEDs.configAllSettings(config, 100);
    }

    public void toggleAnimDirection() {
        CANDLE_animDirection = !CANDLE_animDirection;
    }

    /*
     * these were in the example code, i thought it would be cool to just
     * leave incrementAnimation() and decrementAnimation() in
     */
    public void incrementAnimation() {
        switch(CANDLE_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Empty); break;
            case Empty: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(CANDLE_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Empty); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case Empty: changeAnimation(AnimationTypes.TwinkleOff); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }


    
    /* yknow how it is with them getters and setters :tongue: */
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }
    
    public double getVbat() {
        return LEDs.getBusVoltage();
    }

    public double getCurrent() {
        return LEDs.getCurrent();
    }

    public double getTemperature() {
        return LEDs.getTemperature();
    }

    public AnimationTypes getPreviousAnimation() {
        return CANDLE_previousAnimation;
    }

    public void configBrightness(double percent) {
        LEDs.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        LEDs.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        LEDs.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        LEDs.configStatusLedState(offWhenActive, 0);
    }

    public void changeAnimation(AnimationTypes toChange) {
        if (!(CANDLE_currentAnimation == null)) {
            CANDLE_previousAnimation = CANDLE_currentAnimation;
        }
        
        CANDLE_currentAnimation = toChange;
        
        switch(toChange)
        {
            default:
            case ColorFlow:
                CANDLE_channel = 0;
                CANDLE_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDS_PER_ANIMATION, Direction.Forward, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;
            case Fire:
                CANDLE_channel = 1;
                CANDLE_toAnimate = new FireAnimation(0.5, 0.7, LEDS_PER_ANIMATION, 0.8, 0.5, CANDLE_animDirection, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;
            case Larson:
                CANDLE_channel = 2;
                CANDLE_toAnimate = new LarsonAnimation(0, 255, 46, 0, 0.1, LEDS_PER_ANIMATION, BounceMode.Front, 3, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;
            case Rainbow:
                CANDLE_channel = 3;
                CANDLE_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, CANDLE_animDirection, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;
            case RgbFade:
                CANDLE_channel = 4;
                CANDLE_toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;
            case SingleFade:
                CANDLE_channel = 5;
                CANDLE_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_PER_ANIMATION, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;
            case Strobe:
                CANDLE_channel = 6;
                CANDLE_toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.01, LEDS_PER_ANIMATION, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;
            case Twinkle:
                CANDLE_channel = 7;
                CANDLE_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LEDS_PER_ANIMATION, TwinklePercent.Percent42, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;
            case TwinkleOff:
                CANDLE_channel = 8;
                CANDLE_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_PER_ANIMATION, TwinkleOffPercent.Percent76, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;
            case Empty:
                CANDLE_channel = 9;
                CANDLE_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, CANDLE_animDirection, CANDLE_channel * LEDS_PER_ANIMATION + 8);
                break;

            case SetAll:
                CANDLE_toAnimate = null;
                break;
        }
        System.out.println("Changed to " + CANDLE_currentAnimation.toString());
    }

    public void clearAllAnims() {
        CANDLE_clearAllAnims = true;
    }


    /*
     * 
     * 
     * this is the main command btw haha unnlesssssss :tongue:
     */
    public Command defaultCommand() {
        return runOnce(() -> {
            BatteryIndicator.fullClear();

            MainStrip.setColor(white);
        });
    }
}