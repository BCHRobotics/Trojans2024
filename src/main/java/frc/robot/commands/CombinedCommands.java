package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants.kElevatorPositions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;

public class CombinedCommands {
    Elevator m_elevator;
    Mechanism m_mechanism;

    Command pickupFromGround() {
        return
            Commands.runOnce(() -> m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE))
                .until(Elevator::checkAtGoal)
                .andThen(
                    Commands.runOnce(() -> m_mechanism.groundIntake(6)))
                    .andThen(
                        Commands.runOnce(() -> m_elevator.moveToPositionCommand(kElevatorPositions.TRAVEL)));
    }
}
