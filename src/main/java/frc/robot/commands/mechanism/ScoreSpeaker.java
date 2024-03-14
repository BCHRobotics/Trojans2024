package frc.robot.commands.mechanism;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanism;

public class ScoreSpeaker extends Command {
    private final Mechanism m_mechanism;
    private final double m_speed;
    private final Supplier<Boolean> m_unloaded;

    public ScoreSpeaker(Mechanism mechanism, double speed, Supplier<Boolean> unloaded) {
        m_mechanism = mechanism;
        m_speed = speed;
        m_unloaded = unloaded;

        addRequirements(m_mechanism);
    }
    

    @Override
    public void initialize() {
        m_mechanism.setSourceSpeed(m_speed);
        m_mechanism.setAmpSpeed(m_speed);
        // Delay setting belt speed
        Timer.delay(0.5);
        m_mechanism.setBeltSpeed(-m_speed);
    }


    @Override
    public void end(boolean interrupted) {
        m_mechanism.stopMotors();
    }

    @Override
    public boolean isFinished() {
        // If it's unloaded it will run end(true), and stop the motors.
        boolean unloaded = m_unloaded.get();

        return unloaded == true;
    }
}
