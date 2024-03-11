package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.subsystems.Elevator;
import frc.utils.controllers.BetterProfiledPIDController;

public class MoveToPosition extends Command {
    private final Elevator m_elevator;
    private final BetterProfiledPIDController m_controller;
    private final double m_goal;

    public MoveToPosition(Elevator elevator, ElevatorPositions position) {
        m_elevator = elevator;
        m_controller = elevator.getController();
        m_goal = position.getGoal();

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_controller.setGoal(m_goal);
    }

    @Override
    public void execute() {
        if(m_elevator.limitHit()) {
            m_controller.forceAtGoal();
        }
    }

    @Override
    public boolean isFinished() {
        return m_controller.atGoal();
    }
}
