package frc.utils;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase{
    private final DigitalOutput mFrontLED;
    private final DigitalOutput mRearLeftLED;
    private final DigitalOutput mRearRightLED;

    public LEDs() {
        this.mFrontLED = new DigitalOutput(LEDConstants.kFrontLEDPort);
        this.mRearLeftLED = new DigitalOutput(LEDConstants.kRearLeftLEDPort);
        this.mRearRightLED = new DigitalOutput(LEDConstants.kRearRightLEDPort);
    }

    public Command setLEDs(boolean state) {
        return this.runOnce(
            () -> {
                this.mFrontLED.set(state); 
                this.mRearLeftLED.set(state);
                this.mRearRightLED.set(state);
            }
        );
    }
}
