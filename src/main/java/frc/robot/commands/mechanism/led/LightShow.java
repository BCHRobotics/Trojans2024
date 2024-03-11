package frc.robot.commands.mechanism.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.subsystems.Mechanism;

public class LightShow extends Command {
    private Mechanism m_mechanism;

    public LightShow(Mechanism mechanism) {
        m_mechanism = mechanism;
    }

    @Override
    public void initialize() {
        m_mechanism.powerLEDs(LEDColor.OFF);
    }

    @Override
    public void execute() {
        m_mechanism.powerLEDs(LEDColor.BLUE);
        Timer.delay(0.25);
        m_mechanism.powerLEDs(LEDColor.YELLOW);
        Timer.delay(0.35);
    }
}
