package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.mechanism.EjectGround;
import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.Elevator;
import frc.utils.devices.BeamBreak.Phase;

public class ReleaseToGround extends SequentialCommandGroup {

    public ReleaseToGround(Mechanism mechanism, Elevator elevator) {
        addCommands(
            new EjectGround(mechanism, 12, mechanism.phaseChecker(Phase.NONE)),
            new MoveToPosition(elevator, ElevatorPositions.AMP),
            new InstantCommand(() -> mechanism.setColor(LEDColor.OFF))
        );
    }
}
