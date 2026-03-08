package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

public class LightPollution extends SubsystemBase {
    enum PatternType {
        SOLID,
        SOLID_BLINK,
        RAINBOW,
        SCROLLING_RAINBOW
    }
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private PatternType patternType = PatternType.SCROLLING_RAINBOW;
    private LEDPattern ledPattern;
    
    public LightPollution(int pwmPort, int length) {
        led = new AddressableLED(pwmPort);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
        setModeRollingRainbow();
    }

    public PatternType getPatternType() {
        return patternType;
    }

    public void setModeRainbow() {
        patternType = PatternType.RAINBOW;
        ledPattern = LEDPattern.rainbow(255, 255);
    }
    
    public void setModeRollingRainbow() {
        patternType = PatternType.SCROLLING_RAINBOW;
        var rainbow = LEDPattern.rainbow(255, 255);
        ledPattern = rainbow.scrollAtRelativeSpeed(Frequency.ofBaseUnits(0.5, Units.Hertz));;
    }

    public void setModeSolid(Color color) {
        patternType = PatternType.SOLID;
        ledPattern = LEDPattern.solid(getSwappedColor(color));
    }

    public void setModeSolidBlink(Color color) {
        patternType = PatternType.SOLID_BLINK;
        ledPattern =  LEDPattern.solid(getSwappedColor(color)).blink(Seconds.of(0.25));
    }

    private Color getSwappedColor(Color color) {
        return new Color(color.red, color.blue, color.green);
    }

    private int ledUpdateCounter = 0;

    @Override
    public void periodic() {
        // 每 3 個迴圈 (約 60ms) 才更新一次 LED，減少 periodic 迴圈負擔
        if (++ledUpdateCounter >= 3) {
            ledPattern.applyTo(buffer);
            led.setData(buffer);
            ledUpdateCounter = 0;
        }
    }
}
