package frc.robot.commands.combined.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.mechanism.ScoreAmp;
import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.Elevator;
import frc.utils.devices.BeamBreak.Phase;

public class ScoreSpeakerCmd extends SequentialCommandGroup {
    /**
     * Scores a note into the speaker.
     * <p> REPLACES: scoreSpeaker
     */
    public ScoreSpeakerCmd(Mechanism mechanism, Elevator elevator) {
        addCommands(
            new InstantCommand(() -> {
                mechanism.setSourceSpeed(12);
                mechanism.setAmpSpeed(12);
            }),
            new ParallelRaceGroup(
                new InstantCommand(() -> mechanism.setBeltSpeed(-12)),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.NONE)
            ),
            new InstantCommand(() -> mechanism.stopMotors()),
            new WaitCommand(0.1),
            new MoveToPosition(elevator, ElevatorPositions.INTAKE),
            new InstantCommand(() -> mechanism.powerLEDs(LEDColor.OFF))
        );
    }
}
