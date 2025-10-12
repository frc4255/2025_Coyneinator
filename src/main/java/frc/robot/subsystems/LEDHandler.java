package frc.robot.subsystems;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.led.LedColor;
import frc.robot.led.LedPattern;

/**
 * Thin wrapper around the CTRE CANdle LED controller.
 */
public class LEDHandler extends SubsystemBase {
    private final CANdle candle;
    private final boolean hasHardware;
    private final int ledCount;

    private LedPattern activePattern;
    private LedColor lastAppliedColor;
    private boolean toggleState = true;
    private double lastToggleTimestamp = 0.0;
    private boolean disabledAnimationActive = false;

    public LEDHandler(int deviceID, int ledCount) {
        this.ledCount = ledCount;
        if (RobotBase.isReal()) {
            candle = new CANdle(deviceID);
            CANdleConfiguration config = new CANdleConfiguration();
            config.statusLedOffWhenActive = true;
            config.disableWhenLOS = false;
            config.stripType = LEDStripType.GRB;
            config.brightnessScalar = 0.5;
            config.vBatOutputMode = VBatOutputMode.Modulated;
            candle.configAllSettings(config, 100);
            hasHardware = true;
        } else {
            candle = null;
            hasHardware = false;
        }
    }

    public void setPattern(LedPattern pattern) {
        activePattern = pattern;
        toggleState = true;
        lastToggleTimestamp = Timer.getFPGATimestamp();
        if (!disabledAnimationActive) {
            if (pattern == null) {
                applyColor(LedColor.BLACK);
            } else {
                applyColor(pattern.primary());
            }
        }
    }

    public void clearPattern() {
        setPattern(null);
    }

    public void setDisabledAnimation(boolean enabled) {
        if (enabled && !disabledAnimationActive) {
            disabledAnimationActive = true;
            lastAppliedColor = null;
            if (hasHardware) {
                candle.animate(new RainbowAnimation(0.5, 0.5, ledCount));
            }
        } else if (!enabled && disabledAnimationActive) {
            disabledAnimationActive = false;
            if (hasHardware) {
                candle.clearAnimation(0);
            }
            if (activePattern == null) {
                applyColor(LedColor.BLACK);
            } else {
                applyColor(activePattern.primary());
            }
        }
    }

    @Override
    public void periodic() {
        if (disabledAnimationActive) {
            return;
        }

        if (activePattern == null) {
            applyColor(LedColor.BLACK);
            return;
        }

        if (activePattern.style() == LedPattern.Style.SOLID) {
            applyColor(activePattern.primary());
            return;
        }

        double period = activePattern.periodSeconds();
        if (period <= 0.0) {
            period = 0.5;
        }
        double now = Timer.getFPGATimestamp();
        if ((now - lastToggleTimestamp) >= (period / 2.0)) {
            toggleState = !toggleState;
            lastToggleTimestamp = now;
        }

        LedColor color;
        if (activePattern.style() == LedPattern.Style.FLASHING) {
            color = toggleState ? activePattern.primary() : LedColor.BLACK;
        } else {
            color = toggleState ? activePattern.primary() : activePattern.secondary();
        }
        applyColor(color);
    }

    private void applyColor(LedColor color) {
        Objects.requireNonNull(color, "color");
        if (lastAppliedColor != null && lastAppliedColor.equals(color)) {
            return;
        }

        if (hasHardware) {
            int[] rgb = color.toArray();
            candle.setLEDs(rgb[0], rgb[1], rgb[2]);
        }
        lastAppliedColor = color;
        Logger.recordOutput("LEDs/RGB", new int[] {color.r(), color.g(), color.b()});
    }
}
