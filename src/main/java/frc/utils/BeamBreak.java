package frc.utils;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.MechanismConstants;

public class BeamBreak {
    // Digital inputs for the beam break sensors
    private final DigitalInput m_pickupSensor;
    private final DigitalInput m_loadedSensor;
    private final DigitalInput m_shootSensor;

    private static Solenoid PCMChannel0;
    private static Solenoid PCMChannel1;
    private static Solenoid PCMChannel2;
    private static Solenoid PCMChannel3;
    private static Solenoid PCMChannel6;
    private static Solenoid PCMChannel7;

    // Enum for the different phases
    public enum Phase {
        NONE, PICKUP, LOADED, SHOOT
    }

    // Current phase
    private Phase m_currentPhase;

    public BeamBreak(int pickupSensorChannel, int loadedSensorChannel, int shootSensorChannel) {
        m_pickupSensor = new DigitalInput(MechanismConstants.kPickupSensorChannel);
        m_loadedSensor = new DigitalInput(MechanismConstants.kLoadedSensorChannel);
        m_shootSensor = new DigitalInput(MechanismConstants.kShootSensorChannel);

        PCMChannel0 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        PCMChannel1 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        PCMChannel2 = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        PCMChannel3 = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
        PCMChannel6 = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
        PCMChannel7 = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

        m_currentPhase = Phase.NONE; // Default phase
    }

    public static void solenoidChannelActive(boolean activeChannel) {
        PCMChannel0.set(activeChannel);
        PCMChannel1.set(activeChannel);
        PCMChannel2.set(activeChannel);
        PCMChannel3.set(activeChannel);
        PCMChannel6.set(activeChannel);
        PCMChannel7.set(activeChannel);
    }

    /**
     * Uupdate the phase based on sensor inputs
     */
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

    /**
     * Gets the current phase
     * @return the phase
     */
    public Phase getPhase() {
        return m_currentPhase;
    }
}
