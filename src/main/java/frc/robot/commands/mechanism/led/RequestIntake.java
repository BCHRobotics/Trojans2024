package frc.robot.commands.mechanism.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.subsystems.Mechanism;
import frc.utils.devices.BeamBreak.Phase;

public class RequestIntake extends Command {
    private Mechanism m_mechanism;
    private LEDColor m_color;

    public RequestIntake(Mechanism mechanism, LEDColor color) {
        m_mechanism = mechanism;
        m_color = color;
    }

    @Override
    public void initialize() {
        if(m_mechanism.getPhase() != Phase.NONE || m_mechanism.getColor() == m_color) {
            m_mechanism.setColor(LEDColor.OFF);
        }  else {
            m_mechanism.setColor(m_color);
        }
    }
}
