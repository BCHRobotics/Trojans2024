package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.kElevatorPositions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;

public class CombinedCommands {
    Elevator m_elevator;
    Mechanism m_mechanism;

    public CombinedCommands() {
        m_elevator = Elevator.getInstance();
        m_mechanism = Mechanism.getInstance();
    }

    public Command pickupFromSource() {
        return m_elevator.moveToPositionCommand(kElevatorPositions.SOURCE)
                    .andThen(m_mechanism.sourceIntake(6));
    }

    public Command scoreIntoSpeaker() {
        return m_elevator.moveToPositionCommand(kElevatorPositions.AMP)
                    .andThen(m_mechanism.scoreSpeaker(12));
    }
}
