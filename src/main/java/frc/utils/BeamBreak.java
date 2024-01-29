package frc.utils;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.MechanismConstants;

public class BeamBreak {
    // Digital inputs for the beam break sensors
    private final DigitalInput m_pickupSensor;
    private final DigitalInput m_loadedSensor;
    private final DigitalInput m_shootSensor;

    // Enum for the different phases
    public enum Phase {
        NONE, PICKUP, LOADED, SHOOT
    }

    // Current phase
    private Phase m_currentPhase;

    // Constructor
    public BeamBreak(int pickupSensorChannel, int loadedSensorChannel, int shootSensorChannel) {
        m_pickupSensor = new DigitalInput(MechanismConstants.kPickupSensorChannel);
        m_loadedSensor = new DigitalInput(MechanismConstants.kLoadedSensorChannel);
        m_shootSensor = new DigitalInput(MechanismConstants.kShootSensorChannel);

        m_currentPhase = Phase.NONE; // Default phase
    }

    // Method to update the phase based on sensor inputs
    public void updatePhase() {
        if (!m_loadedSensor.get()) {
            m_currentPhase = Phase.LOADED;

        } else if (!m_pickupSensor.get()) {
            m_currentPhase = Phase.PICKUP;

        } else if (!m_shootSensor.get()) {
            m_currentPhase = Phase.SHOOT;

        } else if (m_loadedSensor.get() 
                && m_pickupSensor.get() 
                && m_shootSensor.get()) {
            m_currentPhase = Phase.NONE;
        }
    }

    public Phase getPhase() {
        return m_currentPhase;
    }
}
