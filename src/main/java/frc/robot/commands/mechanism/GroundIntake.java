package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Mechanism;

public class GroundIntake extends Command {
    private final Mechanism m_mechanism;
    private final double m_speed;
    private final double m_slowPercent;
    private final boolean m_isAuto;

    public GroundIntake(Mechanism mechanism, double speed, boolean isAuto) {
        this(mechanism, speed, 1, isAuto);
    }


    public GroundIntake(Mechanism mechanism, double speed, double slowPercent, boolean isAuto) {
        m_mechanism = mechanism;
        m_speed = speed;
        m_slowPercent = slowPercent;
        m_isAuto = isAuto;

        addRequirements(m_mechanism);
    }
    

    @Override
    public void initialize() {
        if(m_isAuto) {
            m_mechanism.setBeltSpeed(-m_speed * m_slowPercent);
            m_mechanism.setSourceSpeed(m_speed * m_slowPercent * 0.25);
            m_mechanism.setAmpSpeed(m_speed * m_slowPercent * 0.25);
        } else {
            m_mechanism.setBeltSpeed(-m_speed * m_slowPercent);
            m_mechanism.setSourceSpeed(m_speed * m_slowPercent);
            m_mechanism.setAmpSpeed(m_speed * m_slowPercent);
        }
    }


    @Override
    public void end(boolean interrupted) {
        m_mechanism.stopMotors();
    }
}
