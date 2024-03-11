package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;

public class CombinedCommands {
    Elevator m_elevator;
    Mechanism m_mechanism;

    public CombinedCommands() {
        m_elevator = Elevator.getInstance();
        m_mechanism = Mechanism.getInstance();
    }

    // public Command pickupFromSource() {
    //     return m_elevator.moveToPositionCommand(ElevatorPositions.SOURCE)
    //                 .andThen(m_mechanism.sourceIntake(6));
    // }

    // public Command scoreIntoSpeaker() {
    //     return m_elevator.moveToPositionCommand(ElevatorPositions.AMP)
    //                 .andThen(m_mechanism.scoreSpeaker(12))
    //                 .beforeStarting(new WaitCommand(0.1));
                    
    // }

    // public Command scoreIntoAmp() {
    //     return m_elevator.moveToPositionCommand(ElevatorPositions.AMP)
    //                 .andThen(new WaitCommand(0.9))
    //                 .andThen(m_mechanism.scoreAmp(6))
    //                 .beforeStarting(new WaitCommand(0.1));
                    
    // }
}
