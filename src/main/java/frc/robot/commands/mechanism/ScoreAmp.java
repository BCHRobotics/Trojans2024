package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Mechanism;

public class ScoreAmp extends Command {
    private final Mechanism m_mechanism;
    private final double m_speed;


    public ScoreAmp(Mechanism mechanism, double speed) {
        m_mechanism = mechanism;
        m_speed = speed;


        addRequirements(m_mechanism);
    }

    @Override
    public void initialize() {
        m_mechanism.setBeltSpeed(-m_speed);
        m_mechanism.setSourceSpeed(m_speed);
        m_mechanism.setAmpSpeed(-m_speed * 0.7);
    }

    @Override
    public void end(boolean interrupted) {
        m_mechanism.stopMotors();
    }
}
