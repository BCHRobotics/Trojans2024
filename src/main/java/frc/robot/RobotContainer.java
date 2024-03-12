// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.combined.auto.GroundIntakeAutoCmd;
import frc.robot.commands.combined.auto.GroundReleaseAutoCmd;
import frc.robot.commands.combined.teleop.GroundIntakeCmd;
import frc.robot.commands.combined.teleop.ScoreAmpCmd;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mechanism;
import frc.utils.devices.BeamBreak;
import frc.utils.devices.BeamBreak.Phase;
import frc.utils.devices.Camera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.mechanism.led.LightShow;
import frc.robot.commands.mechanism.led.RequestIntake;

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
    private final CombinedCommands m_combinedCommands = new CombinedCommands();
    private final Camera m_photonCam = new Camera(VisionConstants.kNoteCameraName);

    // Flightstick controller
    //CommandJoystick m_driverFlightstickController = new CommandJoystick(OIConstants.kFlightstickPort);
    // Driving controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDrivingControllerXBoxPort);
    // Operator controller
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatingControllerXBoxPort);

    // The auto chooser
    private final SendableChooser<Command> autoChooser;
    // The input method chooser
    private final SendableChooser<Boolean> inputChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        configureNamedCommands();

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
        this.configureDefaultCommands(false);

        BeamBreak.solenoidChannelActive(true);
    }

    // Configures default commands
    public void configureDefaultCommands(boolean isRedAlliance) {
        final double invert = isRedAlliance ? -1 : 1;

        m_robotDrive.setAlliance(isRedAlliance);
        
        // Configure default commands
        if (inputChooser.getSelected().booleanValue() == true) {
            // Configure the drivetrain to use the flightstick
            // m_robotDrive.setDefaultCommand(
            //     // Joystick movement controls robot movement (up, right, left, down).
            //     // Turning is controlled by the twist axis of the flightstick.
            //     new RunCommand(
            //         () -> m_robotDrive.driveCommand(
            //             -MathUtil.applyDeadband(m_driverFlightstickController.getY(), OIConstants.kDriveDeadband) * invert,
            //             -MathUtil.applyDeadband(m_driverFlightstickController.getX(), OIConstants.kDriveDeadband) * invert,
            //             -MathUtil.applyDeadband(m_driverFlightstickController.getTwist(), OIConstants.kTwistDeadband),
            //             OIConstants.kFieldRelative, OIConstants.kRateLimited, !m_mechanism.checkState(Phase.NONE)),
            //         m_robotDrive));
        } else {
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
    }

    public void configureNamedCommands() {
        // Apriltag alignment command for amp
        NamedCommands.registerCommand("ALIGN TAG", new RunCommand(
            () -> m_robotDrive.driveToTag(VisionConstants.kAmpOffsetX, VisionConstants.kAmpOffsetY)).until( // Run the alignwithtag function
                () -> m_robotDrive.checkAlignment()).beforeStarting( // Stop when checkAlignment is true
                    new InstantCommand(
                        () -> m_robotDrive.alignWithTag())).alongWith(
                            new MoveToPosition(m_elevator, ElevatorPositions.AMP)).andThen(
                                new ScoreAmpCmd(m_mechanism, m_elevator))); // Set alignmode to true before starting

        // Apriltag alignment command for speaker
        // NOT USED
        NamedCommands.registerCommand("ALIGN SPEAKER", new RunCommand(
            () -> m_robotDrive.driveToTag(VisionConstants.kSpeakerOffsetX, VisionConstants.kSpeakerOffsetY)).until( // Run the alignwithtag function
                () -> m_robotDrive.checkAlignment()).beforeStarting( // Stop when checkAlignment is true
                    new InstantCommand(
                        () -> m_robotDrive.alignWithTag())).alongWith(
                            new MoveToPosition(m_elevator, ElevatorPositions.AMP)).andThen(
                                new ScoreAmpCmd(m_mechanism, m_elevator))); // Set alignmode to true before starting

        // Note alignment command
        NamedCommands.registerCommand("ALIGN NOTE", new RunCommand(
            () -> m_robotDrive.driveToNote()).until( // Run the 'drive to note' function
                () -> m_mechanism.checkState(Phase.GROUND_PICKUP)).beforeStarting( // Stop when checkAlignment is true, i.e the robot is done aligning
                    new InstantCommand(
                        () -> m_robotDrive.alignWithNote())).alongWith(
                            new MoveToPosition(m_elevator, ElevatorPositions.INTAKE)).alongWith(
                                new GroundIntakeAutoCmd(m_mechanism)).andThen(new InstantCommand(() -> m_robotDrive.cancelAlign()))); // Set alignmode to true before starting, and set isAligned to false

        // A command for canceling the current align command
        NamedCommands.registerCommand("CANCEL ALIGN", new InstantCommand(() -> m_robotDrive.cancelAlign()));

        NamedCommands.registerCommand("INTAKE", new GroundIntakeAutoCmd(m_mechanism));
        NamedCommands.registerCommand("RELEASE", new GroundReleaseAutoCmd(m_mechanism, m_elevator));
        NamedCommands.registerCommand("AMP SCORE", new ScoreAmpCmd(m_mechanism, m_elevator));
        NamedCommands.registerCommand("ELEVATOR LOW", new MoveToPosition(m_elevator, ElevatorPositions.INTAKE));
        NamedCommands.registerCommand("ELEVATOR HIGH", new MoveToPosition(m_elevator, ElevatorPositions.AMP));
        NamedCommands.registerCommand("SPEAKER SCORE", m_combinedCommands.scoreIntoSpeaker(m_mechanism, m_elevator));
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

        configureButtonBindingsDriverXbox();

        configureButtonBindingsOperator();

        configureButtonBindingsFlightstick();

    }

    /**
     * Binding for driver xbox controller buttons
     */
    private void configureButtonBindingsDriverXbox() {
        // Zero heading command (Right Trigger)
        //this.m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        // Brake command (Left Trigger)
        this.m_driverController.leftTrigger().whileTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));
        // Slow mode command (Left Bumper)
        this.m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true), m_robotDrive));
        this.m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false), m_robotDrive));

        this.m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_robotDrive.setFastMode(true), m_robotDrive));
        this.m_driverController.rightBumper().onFalse(new InstantCommand(() -> m_robotDrive.setFastMode(false), m_robotDrive));

        // Align with tag
        this.m_driverController.x().onTrue(new InstantCommand(() -> m_robotDrive.alignWithTag()));
        // Align with note
        this.m_driverController.b().onTrue(new InstantCommand(() -> m_robotDrive.alignWithNote()));
        // Align with speaker
        //this.m_driverController.y().onTrue(new InstantCommand(() -> m_robotDrive.alignWithSpeaker()));
        this.m_driverController.y().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        // Cancel Alignment
        this.m_driverController.a().onTrue(new InstantCommand(() -> m_robotDrive.cancelAlign()));

        this.m_driverController.povLeft().whileTrue(new LightShow(m_mechanism));
    }

    /**
     * Binding for flightstick controller buttons
     */
    private void configureButtonBindingsFlightstick() {
        // // Zero heading command (Button 5)
        // this.m_driverFlightstickController.button(5).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        // // Brake command (Button 1)
        // this.m_driverFlightstickController.button(1).whileTrue(new RunCommand(() -> m_robotDrive.setX(),m_robotDrive));
        // // Slow mode command (Button 2)
        // this.m_driverFlightstickController.button(2).onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true), m_robotDrive));
        // this.m_driverFlightstickController.button(2).onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false), m_robotDrive));
        // // Align with tag (Button 3)
        // this.m_driverFlightstickController.button(3).onTrue(new InstantCommand(() -> m_robotDrive.alignWithTag()));
        // // Align with note (Button 4)
        // this.m_driverFlightstickController.button(4).onTrue(new InstantCommand(() -> m_robotDrive.alignWithNote()));
        // // Cancel Alignment (Button 6)
        // this.m_driverFlightstickController.button(6).onTrue(new InstantCommand(() -> m_robotDrive.cancelAlign()));
    }

    /**
     * Binding for operator xbox controller buttons
     */
    private void configureButtonBindingsOperator() {
        // Moving the elevator
        this.m_operatorController.povUp().onTrue(new MoveToPosition(m_elevator, ElevatorPositions.AMP));
        this.m_operatorController.povRight().onTrue(new MoveToPosition(m_elevator, ElevatorPositions.SOURCE));
        this.m_operatorController.povDown().onTrue(new MoveToPosition(m_elevator, ElevatorPositions.INTAKE));
        // Request intake (ground and source)
        this.m_operatorController.leftBumper().onTrue(new RequestIntake(m_mechanism, LEDColor.PURPLE));
        this.m_operatorController.rightBumper().onTrue(new RequestIntake(m_mechanism, LEDColor.CYAN));

        // Scoring
        this.m_operatorController.b().onTrue(new ScoreAmpCmd(m_mechanism, m_elevator));
        //this.m_operatorController.povLeft().onTrue(this.m_mechanism.scoreSpeaker(12));
        // Intaking
        //this.m_operatorController.y().onTrue(this.m_mechanism.sourceIntake(6));
        this.m_operatorController.x().onTrue(new GroundIntakeCmd(m_mechanism));
        // Cancel command
        this.m_operatorController.a().onTrue(new InstantCommand(() -> m_mechanism.stopMotors(), m_mechanism));

        this.m_operatorController.y().onTrue(this.m_combinedCommands.pickupFromSource(m_mechanism, m_elevator));
        this.m_operatorController.povLeft().onTrue(this.m_combinedCommands.scoreIntoSpeaker(m_mechanism, m_elevator));

        this.m_operatorController.leftTrigger().onTrue(new GroundReleaseAutoCmd(m_mechanism, m_elevator));
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
        m_robotDrive.setIdleStates(IdleMode.kBrake);
    }

    public void updateOdometry() {
        m_robotDrive.updateOdometry();
    }

    public void updateVisionMeasurements() {
        Optional<EstimatedRobotPose> visionEst = m_photonCam.getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    Pose2d estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    Matrix<N3, N1> estStdDevs = m_photonCam.getEstimationStdDevs(estPose);

                    m_robotDrive.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
    }
}
