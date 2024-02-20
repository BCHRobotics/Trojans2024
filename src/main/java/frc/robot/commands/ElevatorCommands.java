package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommands extends Elevator{
    Elevator m_elevator;

    public ElevatorCommands() {
        super();
    }

    /**
     * Stops the elevator
     * @return the command for stopping the elevator
     */
    public Command stopElevatorCommand() {
        return Commands.runOnce(() -> cancelAllElevatorCommands());
    }

    //TODO: choose between TOP or BOTTOM position for encoder reset
    //TODO: choose what to do on default
    //TODO: Top and bottom are climb
    /**
     * Sets the elevator positions
     * @param position the position to be set
     * @return the command to get to the position
     */
    public Command moveToPositionCommand(ElevatorConstants.kElevatorPositions position) {
        switch (position) {
            case TOP:
                return null;
                
            case SOURCE:
                return Commands.runOnce(() -> Elevator.m_controller.setGoal(
                    ElevatorConstants.kElevatorGoals[
                    ElevatorConstants.kElevatorPositions.SOURCE.ordinal()]));

            case AMP:
                return Commands.runOnce(() -> Elevator.m_controller.setGoal(
                    ElevatorConstants.kElevatorGoals[
                    ElevatorConstants.kElevatorPositions.AMP.ordinal()]));

            case TRAVEL:
                return Commands.runOnce(() -> Elevator.m_controller.setGoal(
                    ElevatorConstants.kElevatorGoals[
                    ElevatorConstants.kElevatorPositions.TRAVEL.ordinal()]));

            case INTAKE:
                return Commands.runOnce(() -> Elevator.m_controller.setGoal(
                    ElevatorConstants.kElevatorGoals[
                    ElevatorConstants.kElevatorPositions.INTAKE.ordinal()]));

            case BOTTOM:
                return null;

            default:
                return null;
        }
    }
}
