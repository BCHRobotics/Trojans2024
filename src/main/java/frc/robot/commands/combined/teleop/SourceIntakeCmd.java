package frc.robot.commands.combined.teleop;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.mechanism.SourceIntake;
import frc.robot.commands.mechanism.led.ConfirmIntake;
import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.Elevator;
import frc.utils.devices.BeamBreak.Phase;

public class SourceIntakeCmd extends SequentialCommandGroup {
    /**
     * Gets the note from the source and moves the elevator.
     * <p> REPLACES: sourceIntake
     */
    public SourceIntakeCmd(Mechanism mechanism, Elevator elevator) {
        addCommands(
            new ParallelRaceGroup(
                new SourceIntake(mechanism, 6),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.SOURCE_INTAKE)
            ),
            new ParallelRaceGroup(
                new SourceIntake(mechanism, 6, 0.75),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.LOADED)
            ),
            new MoveToPosition(elevator, ElevatorPositions.INTAKE),
            new ConfirmIntake(mechanism)
        );
    }
}
