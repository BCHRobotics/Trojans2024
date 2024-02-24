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

    private static Solenoid PCMChannel0;
    private static Solenoid PCMChannel1;
    private static Solenoid PCMChannel2;
    private static Solenoid PCMChannel3;
    private static Solenoid PCMChannel4;
    private static Solenoid PCMChannel5;
    private static Solenoid PCMChannel6;
    private static Solenoid PCMChannel7;

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

        PCMChannel0 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        PCMChannel1 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        PCMChannel2 = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        PCMChannel3 = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
        PCMChannel4 = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
        PCMChannel5 = new Solenoid(PneumaticsModuleType.CTREPCM,5);
        PCMChannel6 = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
        PCMChannel7 = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

        m_currentPhase = Phase.NONE; // Default phase
    }

    public static void solenoidChannelActive(boolean activeChannel) {
        PCMChannel0.set(activeChannel);
        PCMChannel1.set(activeChannel);
        PCMChannel2.set(activeChannel);
        PCMChannel3.set(activeChannel);
        PCMChannel4.set(activeChannel);
        PCMChannel5.set(activeChannel);
        PCMChannel6.set(activeChannel);
        PCMChannel7.set(activeChannel);
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
