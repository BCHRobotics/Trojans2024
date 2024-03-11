package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.mechanism.GroundIntake;
import frc.robot.commands.mechanism.led.ConfirmIntake;
import frc.robot.subsystems.Mechanism;
import frc.utils.devices.BeamBreak.Phase;

public class GroundIntakeCmd extends SequentialCommandGroup {
    /**
     * Releases the stored note and moves the elevator to intake.
     * <p> REPLACES: groundIntake
     */
    public GroundIntakeCmd(Mechanism mechanism) {
        addCommands(
            new ParallelRaceGroup(
                new GroundIntake(mechanism, 12),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.NONE)
            ),
            new ParallelRaceGroup(
                new GroundIntake(mechanism, 12, 0.75),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.LOADED)
            ),
            new ConfirmIntake(mechanism)
        );
    }
}
