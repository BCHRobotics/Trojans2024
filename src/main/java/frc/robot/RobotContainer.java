// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.equation.IntegerSequence.Combined;
import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants.kElevatorPositions;
import frc.robot.commands.CombinedCommands;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;
import frc.utils.BeamBreak;
import frc.utils.BeamBreak.Phase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drivetrain m_robotDrive = new Drivetrain();
    private final Elevator m_elevator = new Elevator();
    private final Mechanism m_mechanism = new Mechanism();
    private final CombinedCommands m_combinedCommands = new CombinedCommands(m_elevator, m_mechanism);

    // Flightstick controller
    CommandJoystick m_driverFlightstickController = new CommandJoystick(OIConstants.kFlightstickPort);
    // XBox controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kXBoxPort);
    // Operator controller
    CommandXboxController m_operatorController = new CommandXboxController(1);

    // The auto chooser
    private final SendableChooser<Command> autoChooser;
    // The input method chooser
    private final SendableChooser<Boolean> inputChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Apriltag alignment command
        NamedCommands.registerCommand("ALIGN TAG", new RunCommand(
            () -> m_robotDrive.driveToTag(VisionConstants.kAmpOffsetX, VisionConstants.kAmpOffsetY)).until( // Run the alignwithtag function
                () -> m_mechanism.checkState(Phase.GROUND_PICKUP)).beforeStarting( // Stop when checkAlignment is true
                    new InstantCommand(
                        () -> m_robotDrive.alignWithTag())).alongWith(
                            this.m_elevator.moveToPositionCommand(kElevatorPositions.AMP)).andThen(
                                this.m_mechanism.scoreAmp(6))); // Set alignmode to true before starting

        // Note alignment command
        NamedCommands.registerCommand("ALIGN NOTE", new RunCommand(
            () -> m_robotDrive.driveToNote()).until( // Run the 'drive to note' function
                () -> m_robotDrive.checkAlignment()).beforeStarting( // Stop when checkAlignment is true, i.e the robot is done aligning
                    new InstantCommand(
                        () -> m_robotDrive.alignWithNote())).alongWith(
                            this.m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE)).alongWith(
                                this.m_mechanism.groundIntake(12))); // Set alignmode to true before starting, and set isAligned to false

        // A command for canceling the current align command
        NamedCommands.registerCommand("CANCEL ALIGN", new InstantCommand(() -> m_robotDrive.cancelAlign()));

        NamedCommands.registerCommand("INTAKE", m_mechanism.groundIntake(12));
        NamedCommands.registerCommand("AMP SCORE", m_mechanism.scoreAmp(12));
        NamedCommands.registerCommand("ELEVATOR LOW", m_elevator.moveToPositionCommand(kElevatorPositions.AMP));
        NamedCommands.registerCommand("ELEVATOR HIGH", m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE));
        NamedCommands.registerCommand("SPEAKER SCORE", m_mechanism.scoreSpeaker(12));

        inputChooser = new SendableChooser<>();
        // Assigning values to the input method chooser
        inputChooser.addOption("XBoxController", Boolean.FALSE);
        inputChooser.addOption("Flightstick", Boolean.TRUE);
        inputChooser.setDefaultOption("XBoxController", Boolean.FALSE);

        SmartDashboard.putData("Input Chooser", inputChooser);

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        this.configureButtonBindings();
        // Configure the default commands for the input method chosen
        this.configureDefaultCommands();
    }

    // Configures default commands
    public void configureDefaultCommands() {
        // Configure default commands
        if (inputChooser.getSelected().booleanValue() == true) {
            // Configure the drivetrain to use the flightstick
            m_robotDrive.setDefaultCommand(
                // Joystick movement controls robot movement (up, right, left, down).
                // Turning is controlled by the twist axis of the flightstick.
                new RunCommand(
                    () -> m_robotDrive.driveCommand(
                        -MathUtil.applyDeadband(m_driverFlightstickController.getY(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverFlightstickController.getX(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverFlightstickController.getTwist(), OIConstants.kTwistDeadband),
                        OIConstants.kFieldRelative, OIConstants.kRateLimited),
                    m_robotDrive));
        } else {
             // Configure the drivetrain to use the XBox controller
            m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.driveCommand(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kTurnDeadband),
                                OIConstants.kFieldRelative, OIConstants.kRateLimited),
                        m_robotDrive));
        }
    }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
    private void configureButtonBindings() {
        /* 
         * CURRENT BUTTON LAYOUT (subject to change):
         * 
         * -- XBox Controller -- 
         * 
         * Y -- RESET GYRO
         * RIGHT BUMPER -- BRAKE
         * LEFT BUMPER -- TOGGLE SLOW MODE
         * X -- ALIGN WITH TAG
         * B -- ALIGN WITH NOTE
         * A -- CANCEL ALIGN
         * 
         * -- Operator Controller --
         * 
         * POV UP - ELEVATOR TO SOURCE
         * POV RIGHT - ELEVATOR TO AMP
         * POV DOWN - ELEVATOR TO GROUND
         * POV LEFT - LIGHTSHOW (TEMPORARY)
         * LEFT BUMPER - REQUEST GROUND INTAKE
         * RIGHT BUMPER - REQUEST SOURCE INTAKE
         * B - SCORE AMP 
         * Y - SOURCE INTAKE
         * X - GROUND INTAKE
         * A - CANCEL INTAKE
         * 
         * -- Flightstick Controller --
         * 
         * Button 5 - RESET GYRO
         * Button 1 - BRAKE
         * Button 2 - TOGGLE SLOW MODE
         * Button 3 - ALIGN WITH TAG
         * Button 4 - ALIGN WITH NOTE
         * Button 6 - CANCEL ALIGN
         */ 

        /*
         * Driver Controller Buttons
         */

        // Zero heading command (Y Button)
        this.m_driverController.y().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        // Brake command (Right Bumper)
        this.m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));
        // Slow mode command (Left Bumper)
        this.m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true), m_robotDrive));
        this.m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false), m_robotDrive));

        // Align with tag
        this.m_driverController.x().onTrue(new InstantCommand(() -> m_robotDrive.alignWithTag()));
        // Align with note
        // this.m_driverController.b().onTrue(new InstantCommand(() -> m_robotDrive.alignWithNote()));
        // Cancel Alignment
        this.m_driverController.a().onTrue(new InstantCommand(() -> m_robotDrive.cancelAlign()));

        /*
         * Operator Controller Buttons
         */

        // this.m_driverXboxController.a().onTrue(this.m_mechanism.nlightShow());

        // Moving the elevator
        this.m_operatorController.povUp().onTrue(this.m_elevator.moveToPositionCommand(kElevatorPositions.SOURCE));
        this.m_operatorController.povRight().onTrue(this.m_elevator.moveToPositionCommand(kElevatorPositions.AMP));
        this.m_operatorController.povDown().onTrue(this.m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE));
        // Request intake (ground and source)
        this.m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_mechanism.requestIntake(1)));
        this.m_operatorController.rightBumper().onTrue(new InstantCommand(() -> m_mechanism.requestIntake(2)));

        this.m_operatorController.povLeft().onTrue(this.m_mechanism.lightShow());

        //this.m_operatorController.povUp().onTrue(this.m_combinedCommands.pickupFromSource());

        // Scoring
        this.m_operatorController.b().onTrue(this.m_mechanism.scoreAmp(6));
        //this.m_operatorController.rightBumper().onTrue(this.m_mechanism.scoreSpeaker(12));
        // Intaking
        this.m_operatorController.y().onTrue(this.m_mechanism.sourceIntake(6));
        this.m_operatorController.x().onTrue(this.m_mechanism.groundIntake(12));
        // Cancel command
        this.m_operatorController.a().onTrue(this.m_mechanism.stopMechanism());

        /*
         * Flightstick Controller Buttons
         */

        // Zero heading command (Button 5)
        this.m_driverFlightstickController.button(5).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        // // Brake command (Button 1)
        this.m_driverFlightstickController.button(1).onTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));
        // // Toggle slow mode (Button 2)
        this.m_driverFlightstickController.button(2).onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true), m_robotDrive));
        this.m_driverFlightstickController.button(2).onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false), m_robotDrive));

        // // Align with apriltag command (Button 3)
        this.m_driverFlightstickController.button(3).onTrue(new InstantCommand(() -> m_robotDrive.alignWithTag()));
        // // Align with note command (Button 4)
        this.m_driverFlightstickController.button(4).onTrue(new InstantCommand(() -> m_robotDrive.alignWithNote()));
        // // Zero heading command (Button 6)
        this.m_driverFlightstickController.button(6).onTrue(new InstantCommand(() -> m_robotDrive.cancelAlign(), m_robotDrive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */ 
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * This function is called when the robot enters disabled mode, it sets the motors to brake mode.
     */
    public void eStop() {
        m_robotDrive.setIdleStates(1);
    }

    /**
     * enable the PCM channels
     */
    public void enablePCMChannels() {
        BeamBreak.solenoidChannelActive(true);
    }

    /**
     * Sets the speed percentage to use based on the slider on the joystick
     */
    public void setSpeedPercent() {
        // THIS IS COMMENTED OUT FOR XBOX FOR NOW
        //m_robotDrive.setSpeedPercent(1 - ((m_driverController.getThrottle() + 1) / 2));
        m_robotDrive.setSpeedPercent(0.05);
    }

    /**
     * Initializes the LEDs
     */
    public void initLEDs() {
        this.m_mechanism.powerLEDs("Off");
    }
}
