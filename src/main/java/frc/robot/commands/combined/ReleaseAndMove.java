package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.mechanism.GroundRelease;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;

public class ReleaseAndMove extends SequentialCommandGroup {
    /**
     * Releases the stored note and moves the elevator to intake.
     * <p> REPLACES: groundReleaseAuto
     */
    public ReleaseAndMove(Mechanism mechanism, Elevator elevator) {
        addCommands(
            new GroundRelease(mechanism, 12),
            new MoveToPosition(elevator, ElevatorPositions.INTAKE)
        );
    }
}
