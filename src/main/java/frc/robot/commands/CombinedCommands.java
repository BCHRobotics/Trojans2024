package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.ElevatorConstants.kElevatorPositions;
import frc.robot.subsystems.Elevator;

public class CombinedCommands {
    ElevatorCommands m_elevatorCommands;
    IntakeCommands m_intakeCommands;

    Command pickupFromGround() {
        return
            Commands.runOnce(() -> m_elevatorCommands.moveToPositionCommand(kElevatorPositions.INTAKE))
                .until(Elevator::checkAtGoal)
                .andThen(
                    Commands.runOnce(() -> m_intakeCommands.groundIntake(6)))
                    .andThen(
                        Commands.runOnce(() -> m_elevatorCommands.moveToPositionCommand(kElevatorPositions.TRAVEL)));
    }
}
