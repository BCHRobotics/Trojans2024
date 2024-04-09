// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.utils.devices.BeamBreak.Phase;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer{

    // The robot's subsystems and commands are defined here...
    final CommandXboxController driverXbox = new CommandXboxController(0);

    // File Directory With Swerve JSON Files (src/main/deploy/swerve)
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");

    // Creating Swerve Drive 
    SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory);
    
    public RobotContainer() {
        this.configureDefaultCommands(false);
    }

    private void configureDefaultCommands(boolean isRedAlliance) {
    
    // Brake command (Left Trigger)
    this.driverXbox.leftTrigger().whileTrue(new RunCommand(() -> m_swerveSubsystem.setX(), m_swerveSubsystem));
   
    final double invert = isRedAlliance ? -1 : 1;

    

    m_swerveSubsystem.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_swerveSubsystem.driveCommand(
                                -MathUtil.applyDeadband(driverXbox.getLeftY(), OIConstants.kDriveDeadband) * invert,
                                -MathUtil.applyDeadband(driverXbox.getLeftX(), OIConstants.kDriveDeadband) * invert,
                                -MathUtil.applyDeadband(driverXbox.getRightX(), OIConstants.kTurnDeadband)),
                                m_swerveSubsystem));

    }


}
