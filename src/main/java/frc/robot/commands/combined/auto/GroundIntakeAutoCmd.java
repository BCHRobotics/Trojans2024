package frc.robot.commands.combined.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.mechanism.GroundIntake;
import frc.robot.subsystems.Mechanism;
import frc.utils.devices.BeamBreak.Phase;

public class GroundIntakeAutoCmd extends SequentialCommandGroup {
    /**
     * Picks up a note at the ground at a slower rate.
     * <p> REPLACES: groundIntakeAuto
     */
    public GroundIntakeAutoCmd(Mechanism mechanism) {
        addCommands(
            new ParallelRaceGroup(
                new GroundIntake(mechanism, 12, true),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.NONE)
            ),
            new ParallelRaceGroup(
                new GroundIntake(mechanism, 12, 0.75, true),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.LOADED)
            )
        );
    }
}
