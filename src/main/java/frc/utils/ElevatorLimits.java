package frc.utils;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorLimits {
    // Digital inputs for the beam break sensors
    private final DigitalInput m_topLimitSwitch;
    private final DigitalInput m_bottomLimitSwitch;

    // Enum for the different phases
    public enum ElevatorLimit {
        NONE, TOP, BOTTOM
    }

    // Current phase
    private ElevatorLimit m_currentLimitSwitch;

    // Constructor
    public ElevatorLimits(int topLimitPort, int bottomLimitPort) {
        m_topLimitSwitch = new DigitalInput(ElevatorConstants.kTopElevatorLimitSwitchPort);
        m_bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomElevatorLimitSwitchPort);

        m_currentLimitSwitch = ElevatorLimit.TOP;
    }

    // Method to update the phase based on sensor inputs
    public void updateLimit() {
        if (!m_topLimitSwitch.get()) {
            m_currentLimitSwitch = ElevatorLimit.TOP;
            System.out.println("Top limit hit!");

        } else if (!m_bottomLimitSwitch.get()) {
            m_currentLimitSwitch = ElevatorLimit.BOTTOM;
            System.out.println("Bottom limit hit!");

        } else if (m_bottomLimitSwitch.get() 
                && m_topLimitSwitch.get()) {
            m_currentLimitSwitch = ElevatorLimit.NONE;
        }
    }

    public ElevatorLimit getLimit() {
        return m_currentLimitSwitch;
    }
}
