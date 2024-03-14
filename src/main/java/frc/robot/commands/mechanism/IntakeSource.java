package frc.robot.commands.mechanism;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism;
import frc.robot.Constants.MechanismConstants;

public class IntakeSource extends Command {
    private final Mechanism m_mechanism;
    private final double m_speed;
    private final Supplier<Boolean> m_brokeBeam;
    private final Supplier<Boolean> m_isLoaded;

    public IntakeSource(Mechanism mechanism, double speed, Supplier<Boolean> brokeBeam, Supplier<Boolean> isLoaded) {
        m_mechanism = mechanism;
        m_speed = speed;
        m_brokeBeam = brokeBeam;
        m_isLoaded = isLoaded;

        addRequirements(m_mechanism);
    }
    

    @Override
    public void initialize() {
        m_mechanism.setBeltSpeed(m_speed);
        m_mechanism.setSourceSpeed(-m_speed);
        m_mechanism.setAmpSpeed(-m_speed);
    }

    @Override
    public void execute() {
        boolean passedBeam = m_brokeBeam.get();

        if(passedBeam) {
            m_mechanism.setBeltSpeed(m_speed * MechanismConstants.kSlow);
            m_mechanism.setSourceSpeed(-m_speed * MechanismConstants.kSlow);
            m_mechanism.setAmpSpeed(-m_speed * MechanismConstants.kSlow);
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
