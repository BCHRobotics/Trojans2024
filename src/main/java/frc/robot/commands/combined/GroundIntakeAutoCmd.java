package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.mechanism.GroundIntakeAuto;
import frc.robot.subsystems.Mechanism;
import frc.utils.devices.BeamBreak.Phase;

public class GroundIntakeAutoCmd extends SequentialCommandGroup {
    /**
     * Releases the stored note and moves the elevator to intake.
     * <p> REPLACES: groundIntake
     */
    public GroundIntakeAutoCmd(Mechanism mechanism) {
        addCommands(
            new ParallelRaceGroup(
                new GroundIntakeAuto(mechanism, 12),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.NONE)
            ),
            new ParallelRaceGroup(
                new GroundIntakeAuto(mechanism, 12, 0.75),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.LOADED)
            )
        );
    }
}
