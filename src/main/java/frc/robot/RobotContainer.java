// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.Constants.VisionConstants.CameraModes;
import frc.robot.commands.CombinedCommands;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;
import frc.utils.devices.BeamBreak;
import frc.utils.devices.BeamBreak.Phase;
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
    private final Elevator m_elevator;
    private final Mechanism m_mechanism;
    private final CombinedCommands m_combinedCommands = new CombinedCommands();

    // Flightstick controller
    //CommandJoystick m_driverFlightstickController = new CommandJoystick(OIConstants.kFlightstickPort);
    // Driving controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDrivingControllerXBoxPort);
    // Operator controller
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatingControllerXBoxPort);

    // The auto chooser
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_elevator = Elevator.getInstance();
        m_mechanism = Mechanism.getInstance();

        configureNamedCommands();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        this.configureButtonBindings();
        // Configure the default commands for the input method chosen
        this.configureDefaultCommands(false);
    }

    // Configures default commands
    public void configureDefaultCommands(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;

        m_robotDrive.setAlliance(isRedAlliance);
        
        // Configure the drivetrain to use the XBox controller
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                    () -> m_robotDrive.driveCommand(
                            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * invert,
                            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * invert,
                            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kTurnDeadband),
                            OIConstants.kFieldRelative, OIConstants.kRateLimited, !m_mechanism.checkState(Phase.NONE)),
                    m_robotDrive));
    }

    public void configureNamedCommands() {
        // Apriltag alignment command for amp
        NamedCommands.registerCommand("ALIGN TAG", new RunCommand(
            () -> m_robotDrive.driveToTag(CameraModes.AMP.getOffsets()[0], CameraModes.AMP.getOffsets()[1])).until( // Run the alignwithtag function
                () -> m_robotDrive.checkAlignment()).beforeStarting( // Stop when checkAlignment is true
                    new InstantCommand(
                        () -> m_robotDrive.setVisionMode(CameraModes.AMP))).alongWith(
                            this.m_elevator.moveToPositionCommand(ElevatorPositions.AMP)).andThen(
                                this.m_mechanism.scoreAmp(6))); // Set alignmode to true before starting

        // Apriltag alignment command for speaker
        // NOT USED
        NamedCommands.registerCommand("ALIGN SPEAKER", new RunCommand(
            () -> m_robotDrive.driveToTag(CameraModes.AMP.getOffsets()[0], CameraModes.AMP.getOffsets()[1])).until( // Run the alignwithtag function
                () -> m_robotDrive.checkAlignment()).beforeStarting( // Stop when checkAlignment is true
                    new InstantCommand(
                        () -> m_robotDrive.setVisionMode(CameraModes.SPEAKER))).alongWith(
                            this.m_elevator.moveToPositionCommand(ElevatorPositions.AMP)).andThen(
                                this.m_mechanism.scoreAmp(6))); // Set alignmode to true before starting

        // Note alignment command
        NamedCommands.registerCommand("ALIGN NOTE", new RunCommand(
            () -> m_robotDrive.driveToNote()).until( // Run the 'drive to note' function
                () -> m_mechanism.checkState(Phase.GROUND_PICKUP)).beforeStarting( // Stop when checkAlignment is true, i.e the robot is done aligning
                    new InstantCommand(
                        () -> m_robotDrive.setVisionMode(CameraModes.NOTE))).alongWith(
                            this.m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE)).alongWith(
                                this.m_mechanism.groundIntakeAuto(12)).andThen(new InstantCommand(() -> m_robotDrive.cancelAlign()))); // Set alignmode to true before starting, and set isAligned to false

        // A command for canceling the current align command
        NamedCommands.registerCommand("CANCEL ALIGN", new InstantCommand(() -> m_robotDrive.cancelAlign()));

        NamedCommands.registerCommand("INTAKE", m_mechanism.groundIntakeAuto(12));
        NamedCommands.registerCommand("RELEASE", m_mechanism.groundReleaseAuto(12));
        NamedCommands.registerCommand("AMP SCORE", m_mechanism.scoreAmp(6));
        NamedCommands.registerCommand("ELEVATOR LOW", m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE));
        NamedCommands.registerCommand("ELEVATOR HIGH", m_elevator.moveToPositionCommand(ElevatorPositions.AMP));
        NamedCommands.registerCommand("SPEAKER SCORE", m_combinedCommands.scoreIntoSpeaker());
    }

    public void setupAuto() {
        m_robotDrive.cancelAlign();
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
         * LEFT BUMPER - REQUEST GROUND INTAKE
         * RIGHT BUMPER - REQUEST SOURCE INTAKE
         * B - SCORE AMP 
         * Y - SOURCE INTAKE
         * X - GROUND INTAKE
         * A - CANCEL INTAKE
         */ 

        configureButtonBindingsDriver();
        configureButtonBindingsOperator();
    }

    /**
     * Binding for driver xbox controller buttons
     */
    private void configureButtonBindingsDriver() {
        // Zero heading command (Right Trigger)
        //this.m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        // Brake command (Left Trigger)
        this.m_driverController.leftTrigger().whileTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));
        // Slow mode command (Left Bumper)
        this.m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true), m_robotDrive));
        this.m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false), m_robotDrive));

        // Align with tag
        this.m_driverController.x().onTrue(new InstantCommand(() -> m_robotDrive.setVisionMode(CameraModes.AMP)));
        // Align with note
        this.m_driverController.b().onTrue(new InstantCommand(() -> m_robotDrive.setVisionMode(CameraModes.SPEAKER)));
        // Align with speaker
        //this.m_driverController.y().onTrue(new InstantCommand(() -> m_robotDrive.alignWithSpeaker()));
        this.m_driverController.y().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        // Cancel Alignment
        this.m_driverController.a().onTrue(new InstantCommand(() -> m_robotDrive.cancelAlign()));

        this.m_driverController.povLeft().onTrue(this.m_mechanism.lightsOff().andThen(this.m_mechanism.lightShow()));
    }

    /**
     * Binding for operator xbox controller buttons
     */
    private void configureButtonBindingsOperator() {
        // Moving the elevator
        this.m_operatorController.povUp().onTrue(this.m_elevator.moveToPositionCommand(ElevatorPositions.AMP));
        this.m_operatorController.povRight().onTrue(this.m_elevator.moveToPositionCommand(ElevatorPositions.SOURCE));
        this.m_operatorController.povDown().onTrue(this.m_elevator.moveToPositionCommand(ElevatorPositions.INTAKE));
        // Request intake (ground and source)
        this.m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_mechanism.requestIntake(1)));
        this.m_operatorController.rightBumper().onTrue(new InstantCommand(() -> m_mechanism.requestIntake(2)));

        // Scoring
        this.m_operatorController.b().onTrue(m_mechanism.scoreAmp(6));
        //this.m_operatorController.povLeft().onTrue(this.m_mechanism.scoreSpeaker(12));
        // Intaking
        //this.m_operatorController.y().onTrue(this.m_mechanism.sourceIntake(6));
        this.m_operatorController.x().onTrue(this.m_mechanism.groundIntake(12));
        // Cancel command
        this.m_operatorController.a().onTrue(this.m_mechanism.stopMechanism());

        this.m_operatorController.y().onTrue(this.m_combinedCommands.pickupFromSource());
        this.m_operatorController.povLeft().onTrue(this.m_combinedCommands.scoreIntoSpeaker());

        this.m_operatorController.leftTrigger().onTrue(this.m_mechanism.groundReleaseAuto(12));
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
     * Initializes the LEDs
     */
    public void initLEDs() {
        this.m_mechanism.powerLEDs(LEDColor.OFF);
    }
}
