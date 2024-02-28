package frc.utils;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase{
    private final DigitalOutput mFrontLED;
    private final DigitalOutput mRearLED;

    public LEDs() {
        this.mFrontLED = new DigitalOutput(LEDConstants.kFrontLEDPort);
        this.mRearLED = new DigitalOutput(LEDConstants.kRearLEDPort);
    }

    public Command setLEDs(boolean state) {
        return this.runOnce(
            () -> {
                this.mFrontLED.set(state); 
                this.mRearLED.set(state);
            }
        );
    }
}
