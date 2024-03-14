package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.Elevator;
import frc.utils.controllers.BetterProfiledPIDController;

public class MoveToPosition extends Command {
    private final Elevator m_elevator;
    private final BetterProfiledPIDController m_controller;
    private final double m_goal;
    // Forward is true, backwards is false.
    private final boolean m_direction;

    /**
     * Moves the elevator to a specified position.
     * @param elevator The elevator subsystem.
     * @param position The position to travel to.
     */
    public MoveToPosition(Elevator elevator, ElevatorPositions position) {
        m_elevator = elevator;
        m_controller = elevator.getController();
        m_goal = position.getGoal();
        // If the goal is greater we know it's going up, otherwise it isn't.
        m_direction = m_controller.getGoal().position < m_goal;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_controller.setGoal(m_goal);
    }

    @Override
    public void end(boolean interrupted) {
        // If limit is hit we need to force it to reach the goal.
        m_controller.forceAtGoal();
    }

    @Override
    public boolean isFinished() {
        return m_controller.atGoal() || m_elevator.limitHit(m_direction);
    }
}
