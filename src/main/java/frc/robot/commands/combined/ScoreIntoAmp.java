package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.mechanism.ScoreAmp;
import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.Elevator;
import frc.utils.devices.BeamBreak.Phase;

public class ScoreIntoAmp extends SequentialCommandGroup {

    public ScoreIntoAmp(Mechanism mechanism, Elevator elevator) {
        addCommands(
            new MoveToPosition(elevator, ElevatorPositions.AMP),
            new WaitCommand(0.9),
            new ScoreAmp(mechanism, 6, mechanism.phaseChecker(Phase.NONE)),
            new WaitCommand(0.25),
            new MoveToPosition(elevator, ElevatorPositions.INTAKE),
            new InstantCommand(() -> mechanism.setColor(LEDColor.OFF))
        );
    }
}
