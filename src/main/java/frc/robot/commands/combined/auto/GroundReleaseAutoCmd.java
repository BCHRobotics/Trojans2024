package frc.robot.commands.combined.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.mechanism.SourceIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;

public class GroundReleaseAutoCmd extends SequentialCommandGroup {
    /**
     * Releases the stored note and moves the elevator to intake.
     * <p> REPLACES: groundReleaseAuto
     */
    public GroundReleaseAutoCmd(Mechanism mechanism, Elevator elevator) {
        // TODO: Test if this works, because I dont think it will 
        addCommands(
            new SourceIntake(mechanism, 12),
            new MoveToPosition(elevator, ElevatorPositions.INTAKE)
        );
    }
}
