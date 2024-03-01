package frc.utils;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase{
    private final DigitalOutput m_redLED;
    private final DigitalOutput m_greenLED;
    private final DigitalOutput m_blueLED;

    public LEDs() {
        this.m_redLED = new DigitalOutput(LEDConstants.kRedLEDPort);
        this.m_greenLED = new DigitalOutput(LEDConstants.kGreenLEDPort);
        this.m_blueLED = new DigitalOutput(LEDConstants.kBlueLEDPort);
    }

    /**
     * Set the LEDs to a on
     * @param state true for on, false for off
     * @return
     */
    public void setLEDs(boolean red, boolean green, boolean blue) {
        this.m_redLED.set(!red); 
        this.m_greenLED.set(!green);
        this.m_blueLED.set(!blue);
    }
}

/*
 * LED Colour Table
 * 
 *          R       G       B
 * White:   255,    255,    255
 * Red:     255,    0,      0
 * Green:   0,      255,    0
 * Blue:    0,      0,      255
 * Yellow:  255,    255,    0
 * Purple:  255,    0,      255
 * Cyan:    0,      255,    255
 */
