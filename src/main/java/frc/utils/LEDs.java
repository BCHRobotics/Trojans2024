package frc.utils;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase{
    private final DigitalOutput m_FrontLED;
    private final DigitalOutput m_RearLeftLED;
    private final DigitalOutput m_RearRightLED;

    public LEDs() {
        this.m_FrontLED = new DigitalOutput(LEDConstants.kFrontLEDPort);
        this.m_RearLeftLED = new DigitalOutput(LEDConstants.kRearLeftLEDPort);
        this.m_RearRightLED = new DigitalOutput(LEDConstants.kRearRightLEDPort);
    }

    /**
     * Set the LEDs to a on
     * @param state true for on, false for off
     * @return
     */
    public Command setLEDs(boolean state) {
        return this.runOnce(
            () -> {
                this.m_FrontLED.set(state); 
                this.m_RearLeftLED.set(state);
                this.m_RearRightLED.set(state);
            }
        );
    }
}
