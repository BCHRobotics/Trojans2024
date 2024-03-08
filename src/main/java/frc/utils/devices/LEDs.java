package frc.utils.devices;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDColor;

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
     * Set the color of the LEDs using a predefined array
     * @param inputs the LED RGB values (on/off)
     */
    public void setLEDs(LEDColor color) {
        boolean[] inputs = color.getArray();

        this.m_redLED.set(!inputs[0]); 
        this.m_greenLED.set(!inputs[1]);
        this.m_blueLED.set(!inputs[2]); 
    }
}
