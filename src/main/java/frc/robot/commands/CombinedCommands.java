package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants.kElevatorPositions;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;

public class CombinedCommands {
    Elevator m_elevator;
    Mechanism m_mechanism;

    public CombinedCommands(Elevator elevator, Mechanism mechanism) {
        m_elevator = elevator;
        m_mechanism = mechanism;
    }

    public Command pickupFromGround() {
        return
            Commands.runOnce(() -> m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE))
                .until(() -> m_elevator.checkAtGoal())
                .andThen(
                    Commands.runOnce(() -> m_mechanism.groundIntake(12)))
                    .andThen(
                        Commands.runOnce(() -> m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE)));
    }

    public Command pickupFromSource() {
        return
            Commands.run(() -> m_elevator.moveToPositionCommand(kElevatorPositions.SOURCE))
                .andThen(() -> System.out.println("here1"))
                .until(() -> m_elevator.checkAtGoal())
                .andThen(() -> System.out.println("here2"))
                .andThen(
                    Commands.run(() -> m_mechanism.sourceIntake(6)))
                    .andThen(
                        Commands.run(() -> m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE)));
    }

    public Command scoreIntoAmp() {
        return
            Commands.runOnce(() -> m_elevator.moveToPositionCommand(kElevatorPositions.AMP))
                .until(() -> m_elevator.checkAtGoal())
                .andThen(
                    Commands.runOnce(() -> m_mechanism.scoreAmp(6)))
                    .andThen(
                        Commands.runOnce(() -> m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE)));
    }

    public Command scoreIntoSpeaker() {
        return
            Commands.runOnce(() -> m_elevator.moveToPositionCommand(kElevatorPositions.AMP))
                .until(() -> m_elevator.checkAtGoal())
                .andThen(
                    Commands.runOnce(() -> m_mechanism.scoreSpeaker(12)))
                    .andThen(
                        Commands.runOnce(() -> m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE)));
    }
}
