package frc.utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.MechanismConstants;

public class BeamBreak {
    // Digital inputs for the beam break sensors
    private final DigitalInput m_bottomSensor;
    private final DigitalInput m_middleSensor;
    private final DigitalInput m_topSensor;

    private static Solenoid[] PCMChannels = new Solenoid[8];

    // Enum for the different phases
    public enum Phase {
        NONE, GROUND_PICKUP, LOADED, SOURCE_INTAKE
    }

    // Current phase
    private Phase m_currentPhase;

    public BeamBreak() {
        m_bottomSensor = new DigitalInput(MechanismConstants.kBottomSensorChannel);
        m_middleSensor = new DigitalInput(MechanismConstants.kMiddleSensorChannel);
        m_topSensor = new DigitalInput(MechanismConstants.kTopSensorChannel);

        for (int i = 0; i < 8; i++) {
            PCMChannels[i] = new Solenoid(PneumaticsModuleType.CTREPCM, i);
        }

        m_currentPhase = Phase.NONE; // Default phase
    }

    /**
     * Set the solenoid channels to active
     * @param activeChannel
     */
    public static void solenoidChannelActive(boolean activeChannel) {
        for (int i = 0; i < 8; i++) {
            PCMChannels[i].set(activeChannel);
        }
    }

    /**
     * Update the phase based on sensor inputs
     */
    public void updatePhase() {
        m_currentPhase = Phase.NONE;
        if ((!m_middleSensor.get() && !m_bottomSensor.get() && !m_topSensor.get()) 
               || (!m_topSensor.get() && !m_bottomSensor.get())) {
        
            m_currentPhase = Phase.LOADED;
        }
        else if (!m_topSensor.get()) {
            m_currentPhase = Phase.SOURCE_INTAKE;

        } else if (!m_bottomSensor.get()) {
            m_currentPhase = Phase.GROUND_PICKUP;
        }
    }

    /**
     * Gets the current phase
     * @return the phase
     */
    public Phase getPhase() {
        return m_currentPhase;
    }
}
