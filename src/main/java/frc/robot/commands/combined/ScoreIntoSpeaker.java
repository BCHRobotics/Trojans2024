package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.mechanism.ScoreSpeaker;
import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.Elevator;
import frc.utils.devices.BeamBreak.Phase;

public class ScoreIntoSpeaker extends SequentialCommandGroup {

    public ScoreIntoSpeaker(Mechanism mechanism, Elevator elevator) {
        addCommands(
            new MoveToPosition(elevator, ElevatorPositions.AMP),
            new WaitCommand(0.1),
            new ScoreSpeaker(mechanism, 12, mechanism.phaseChecker(Phase.NONE)),
            new WaitCommand(0.1),
            new MoveToPosition(elevator, ElevatorPositions.INTAKE),
            new InstantCommand(() -> mechanism.setColor(LEDColor.OFF))
        );
    }
}
