package frc.robot.commands.mechanism.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.subsystems.Mechanism;

public class ConfirmIntake extends Command {
    private Mechanism m_mechanism;

    public ConfirmIntake(Mechanism mechanism) {
        m_mechanism = mechanism;
    }

    @Override
    public void initialize() {
        m_mechanism.powerLEDs(LEDColor.GREEN);
        Timer.delay(0.1);
        m_mechanism.powerLEDs(LEDColor.OFF);
        Timer.delay(0.1);
        m_mechanism.powerLEDs(LEDColor.GREEN);
        Timer.delay(0.1);
        m_mechanism.powerLEDs(LEDColor.OFF);
        Timer.delay(0.1);
        m_mechanism.powerLEDs(LEDColor.GREEN);
    }
}
