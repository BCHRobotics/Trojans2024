package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Mechanism;
import frc.utils.BeamBreak.Phase;

public class IntakeCommands extends Mechanism {

    public IntakeCommands() {
        super();
    }

    /**
     * A command for intaking from the ground
     * @param speed the commanded speed in voltage [0 --> 12]
     */
    public Command groundIntake(double speed) {
        return parallel(
            Commands.startEnd(() -> this.setBeltSpeed(-speed), () -> this.setBeltSpeed(-speed * 0.75)),
            Commands.startEnd(() -> this.setSourceSpeed(speed), () -> this.setSourceSpeed(speed * 0.75)),
            Commands.startEnd(() -> this.setAmpSpeed(speed), () -> this.setAmpSpeed(speed * 0.75))
        )
        .until(() -> this.checkState(Phase.GROUND_PICKUP))
        .andThen(() -> System.out.println("BOTTOM hit"))
        .andThen(
            parallel(
                Commands.startEnd(() -> this.setBeltSpeed(-speed * 0.75), () -> this.setBeltSpeed(0.0))
                .until(() -> this.checkState(Phase.LOADED))
                .andThen(() -> System.out.println("MIDDLE hit")),

                Commands.startEnd(() -> this.setSourceSpeed(-speed * 0.75), () -> this.setSourceSpeed(0.0))
                .until(() -> this.checkState(Phase.LOADED)),

                Commands.startEnd(() -> this.setAmpSpeed(-speed * 0.75), () -> this.setAmpSpeed(0.0))
                .until(() -> this.checkState(Phase.LOADED))
            )
        );
    }
 
    /**
     * A command for intaking from the source
     * @param speed the commanded speed in voltage [0 --> 12]
     */
    public Command sourceIntake(double speed) {
        return parallel (
            Commands.startEnd(() -> this.setSourceSpeed(-speed), () -> this.setSourceSpeed(-speed * 0.75)),
            Commands.startEnd(() -> this.setAmpSpeed(-speed), () -> this.setAmpSpeed(-speed * 0.75)),
            Commands.startEnd(() -> this.setBeltSpeed(speed), () -> this.setBeltSpeed(speed * 0.75))
        )
        .until(() -> this.checkState(Phase.SOURCE_INTAKE))
        .andThen(
            parallel(
                Commands.startEnd(() -> this.setSourceSpeed(-speed * 0.75), () -> this.setSourceSpeed(0)),
                Commands.startEnd(() -> this.setAmpSpeed(-speed * 0.75), () -> this.setAmpSpeed(0)),
                Commands.startEnd(() -> this.setBeltSpeed(speed * 0.75), () -> this.setBeltSpeed(0))
            )
            .until(() -> this.checkState(Phase.LOADED))
        );
    }

    /**
     * A command for scoring in the amp
     * @param speed the commanded speed in voltage [0 --> 12]
     */
    public Command scoreAmp(double speed) {
        return parallel(
            Commands.startEnd(() -> this.setAmpSpeed(-speed), () -> this.setAmpSpeed(-speed))
            .until(() -> this.checkState(Phase.SOURCE_INTAKE)),

            Commands.startEnd(() -> this.setSourceSpeed(speed), () -> this.setSourceSpeed(speed))
            .until(() -> this.checkState(Phase.SOURCE_INTAKE)),

            Commands.startEnd(() -> this.setBeltSpeed(-speed), () -> this.setBeltSpeed(-speed))
            .until(() -> this.checkState(Phase.SOURCE_INTAKE))
        )
        .andThen(
            parallel(
                Commands.runOnce(() -> this.setAmpSpeed(0)),
                Commands.runOnce(() -> this.setSourceSpeed(0)),
                Commands.runOnce(() -> this.setBeltSpeed(0))
            ).beforeStarting(new WaitCommand(1)));
    }

    public Command newScoreAmp(double speed) {
        return parallel(
            Commands.startEnd(() -> this.setAmpSpeed(-speed), () -> this.setAmpSpeed(-speed))
            .until(() -> this.checkState(Phase.NONE)),

            Commands.startEnd(() -> this.setSourceSpeed(speed), () -> this.setSourceSpeed(speed))
            .until(() -> this.checkState(Phase.NONE)),

            Commands.startEnd(() -> this.setBeltSpeed(-speed), () -> this.setBeltSpeed(-speed))
            .until(() -> this.checkState(Phase.NONE))
        )
        .andThen(
            parallel(
                Commands.runOnce(() -> this.setAmpSpeed(0)),
                Commands.runOnce(() -> this.setSourceSpeed(0)),
                Commands.runOnce(() -> this.setBeltSpeed(0))
            ).beforeStarting(new WaitCommand(3)));
    }

    private boolean checkSpeed() {
        if (this.getBeltSpeed() == 0 && this.getAmpSpeed() == 0 && this.getSourceSpeed() == 0) return true;
        return false;
    }
 
    /**
     * A command for stopping the mechanism
     * @param speed the commanded speed in voltage [0 --> 12]
     */
    public Command stopMechanism() {
        return Commands.runOnce(() -> this.cancelAllMechanismCommands())
            .andThen(
                parallel(
                    Commands.runOnce(() -> this.setBeltSpeed(0)),
                    Commands.runOnce(() -> this.setSourceSpeed(0)),
                    Commands.runOnce(() -> this.setAmpSpeed(0))
                )
            )
            .andThen(
                Commands.runOnce(() -> this.removeAllMechanismCommands())
            );
    }

    public Command nstopMechanism() {
        return Commands.runOnce(() -> this.removeAllMechanismCommands());
    }
}
