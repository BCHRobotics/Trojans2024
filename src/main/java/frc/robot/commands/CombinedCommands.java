package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.combined.teleop.ScoreSpeakerCmd;
import frc.robot.commands.combined.teleop.SourceIntakeCmd;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;

// TODO: Fix this class later
public class CombinedCommands {

    public Command pickupFromSource(Mechanism mechanism, Elevator elevator) {
        return new MoveToPosition(elevator, ElevatorPositions.SOURCE)
                    .andThen(new SourceIntakeCmd(mechanism, elevator));
    }

    public Command scoreIntoSpeaker(Mechanism mechanism, Elevator elevator) {
        return new MoveToPosition(elevator, ElevatorPositions.AMP)
                    .andThen(new ScoreSpeakerCmd(mechanism, elevator))
                    .beforeStarting(new WaitCommand(0.1));
                    
    }

}
