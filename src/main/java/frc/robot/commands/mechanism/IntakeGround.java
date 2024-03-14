package frc.robot.commands.mechanism;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism;
import frc.robot.Constants.MechanismConstants;

public class IntakeGround extends Command {
    private final Mechanism m_mechanism;
    private final double m_speed;
    private final Supplier<Boolean> m_brokeBeam;
    private final Supplier<Boolean> m_isLoaded;
    private final boolean m_isAuto;

    public IntakeGround(Mechanism mechanism, double speed, Supplier<Boolean> brokeBeam, Supplier<Boolean> isLoaded, boolean isAuto) {
        m_mechanism = mechanism;
        m_speed = speed;
        m_brokeBeam = brokeBeam;
        m_isLoaded = isLoaded;
        m_isAuto = isAuto;

        addRequirements(m_mechanism);
    }
    

    @Override
    public void initialize() {
        if(m_isAuto) {
            m_mechanism.setBeltSpeed(-m_speed);
            m_mechanism.setSourceSpeed(m_speed * MechanismConstants.kAutoSlow);
            m_mechanism.setAmpSpeed(m_speed * MechanismConstants.kAutoSlow);
        } else {
            m_mechanism.setBeltSpeed(-m_speed);
            m_mechanism.setSourceSpeed(m_speed);
            m_mechanism.setAmpSpeed(m_speed);
        }
    }

    @Override
    public void execute() {
        boolean passedBeam = m_brokeBeam.get();

        if(passedBeam) {
        // If it's auto we slow it down even more
            if(m_isAuto) {
                m_mechanism.setBeltSpeed(-m_speed * MechanismConstants.kSlow);
                m_mechanism.setSourceSpeed(m_speed * MechanismConstants.kSlow * MechanismConstants.kAutoSlow);
                m_mechanism.setAmpSpeed(m_speed * MechanismConstants.kSlow* MechanismConstants.kAutoSlow);
            } else {
                m_mechanism.setBeltSpeed(-m_speed * MechanismConstants.kSlow);
                m_mechanism.setSourceSpeed(m_speed * MechanismConstants.kSlow);
                m_mechanism.setAmpSpeed(m_speed * MechanismConstants.kSlow);
            }
        }
    }


    @Override
    public void end(boolean interrupted) {
        m_mechanism.stopMotors();
    }

    @Override
    public boolean isFinished() {
        // If it's loaded it will run end(true), and stop the motors.
        boolean isLoaded = m_isLoaded.get();

        return isLoaded == true;
    }
}
