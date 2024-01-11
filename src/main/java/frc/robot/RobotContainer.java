// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.Autos;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final Drivetrain m_robotDrive = new Drivetrain();

    // The driver's controller
    CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);

    // The auto chooser
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        this.configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kTwistDeadband),
                    OIConstants.kFieldRelative, OIConstants.kRateLimited),
                m_robotDrive));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        // Break Command (Button 2)
        m_driverController.button(2).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

        //Zero heading (Button 5)
        m_driverController.button(5).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        
        // Slow Command (Button 1)
        m_driverController.button(1)
            .onTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true), m_robotDrive))
            .onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false), m_robotDrive));    
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return Autos.getBasicAuto(m_robotDrive);
        return autoChooser.getSelected();
    }
}
