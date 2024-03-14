package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.mechanism.IntakeSource;
import frc.robot.commands.mechanism.led.ConfirmIntake;
import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.Elevator;
import frc.utils.devices.BeamBreak.Phase;

public class PickupFromSource extends SequentialCommandGroup {

    public PickupFromSource(Mechanism mechanism, Elevator elevator) {
        addCommands(
            new MoveToPosition(elevator, ElevatorPositions.SOURCE),
            new IntakeSource(mechanism, 6, mechanism.phaseChecker(Phase.SOURCE_INTAKE), mechanism.phaseChecker(Phase.LOADED)),
            new MoveToPosition(elevator, ElevatorPositions.INTAKE),
            new ConfirmIntake(mechanism)
        );
    }
}
