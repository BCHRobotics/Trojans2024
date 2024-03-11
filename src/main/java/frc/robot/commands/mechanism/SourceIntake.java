package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Mechanism;

public class SourceIntake extends Command {
    private final Mechanism m_mechanism;
    private final double m_speed;
    private final double m_slowPercent;

    public SourceIntake(Mechanism mechanism, double speed) {
        this(mechanism, speed, 1);
    }


    public SourceIntake(Mechanism mechanism, double speed, double slowPercent) {
        m_mechanism = mechanism;
        m_speed = speed;
        m_slowPercent = slowPercent;


        addRequirements(m_mechanism);
    }

    @Override
    public void initialize() {
        m_mechanism.setBeltSpeed(m_speed * m_slowPercent);
        m_mechanism.setSourceSpeed(-m_speed * m_slowPercent);
        m_mechanism.setAmpSpeed(-m_speed * m_slowPercent);
    }

    @Override
    public void end(boolean interrupted) {
        m_mechanism.stopMotors();
    }
}
