package frc.robot.commands.combined.teleop;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.mechanism.GroundIntake;
import frc.robot.commands.mechanism.led.ConfirmIntake;
import frc.robot.subsystems.Mechanism;
import frc.utils.devices.BeamBreak.Phase;

public class GroundIntakeCmd extends SequentialCommandGroup {
    /**
     * Picks up the note from the ground and uses LED to confirm intake.
     * <p> REPLACES: groundIntake
     */
    public GroundIntakeCmd(Mechanism mechanism) {
        addCommands(
            new ParallelRaceGroup(
                new GroundIntake(mechanism, 12, false),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.NONE)
            ),
            new ParallelRaceGroup(
                new GroundIntake(mechanism, 12, 0.75, false),
                new WaitUntilCommand(() -> mechanism.getPhase() == Phase.LOADED)
            ),
            new ConfirmIntake(mechanism)
        );
    }
}
