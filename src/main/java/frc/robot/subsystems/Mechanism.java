// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MechanismConstants;
import frc.utils.BeamBreak;
import frc.utils.BeamBreak.Phase;

public class Mechanism extends SubsystemBase{
    private final BeamBreak m_beamBreak = new BeamBreak(
        MechanismConstants.kPickupSensorChannel, 
        MechanismConstants.kLoadedSensorChannel, 
        MechanismConstants.kShootSensorChannel
    );

    private Phase m_currentPhase = Phase.PICKUP;

    private final CANSparkMax m_beltMotor = new CANSparkMax(MechanismConstants.kBeltMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_sourceMotor = new CANSparkMax(MechanismConstants.kSourceMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_ampMotor = new CANSparkMax(MechanismConstants.kAmpMotorCanId, MotorType.kBrushed);

    /** Creates a new Mechanism. */
    public Mechanism() {
        this.m_beltMotor.restoreFactoryDefaults();
        this.m_sourceMotor.restoreFactoryDefaults();
        this.m_ampMotor.restoreFactoryDefaults();

        this.m_beltMotor.setIdleMode(IdleMode.kBrake);
        this.m_sourceMotor.setIdleMode(IdleMode.kBrake);
        this.m_ampMotor.setIdleMode(IdleMode.kBrake);

        this.m_beltMotor.setSmartCurrentLimit(60, 20);
        this.m_sourceMotor.setSmartCurrentLimit(60, 20);
        this.m_ampMotor.setSmartCurrentLimit(60, 20);

        this.m_beltMotor.setInverted(false);
        this.m_sourceMotor.setInverted(false);
        this.m_ampMotor.setInverted(false);

        this.m_beltMotor.setOpenLoopRampRate(0.05);
        this.m_sourceMotor.setOpenLoopRampRate(0.05);
        this.m_ampMotor.setOpenLoopRampRate(0.05);

        this.m_beltMotor.enableVoltageCompensation(12);
        this.m_sourceMotor.enableVoltageCompensation(12);
        this.m_ampMotor.enableVoltageCompensation(12);
    }

    private void updatePhase() {
        this.m_beamBreak.updatePhase();
        this.m_currentPhase = this.m_beamBreak.getPhase();
    }

    /**
     * Sets belt speed in percent output [-1 --> 1]
     */
    private void setBeltSpeed(double speed) {
        this.m_beltMotor.set(speed);
    }

    /**
     * Sets source intake speed in percent output [-1 --> 1]
     */
    private void setSourceSpeed(double speed) {
        this.m_sourceMotor.set(speed);
    }
    
    /**
     * Sets amp motor speed in percent output [-1 --> 1]
     */
    private void setAmpSpeed(double speed) {
        this.m_ampMotor.set(speed);
    }

    public Command scoreAmp(double speed) {
        return parallel(
            Commands.runOnce(() -> {this.setBeltSpeed(speed);}),
            Commands.runOnce(() -> {this.setSourceSpeed(speed);}),
            Commands.runOnce(() -> {this.setAmpSpeed(speed);})
        );
    }

    public Command sourceIntake(double speed) {
        return parallel(
            Commands.runOnce(() -> {this.setBeltSpeed(speed);}),
            Commands.runOnce(() -> {this.setSourceSpeed(speed);}),
            Commands.runOnce(() -> {this.setAmpSpeed(-speed);})
        );
    }

    public Command groundIntake(double speed) {
        return parallel(
            Commands.runOnce(() -> {this.setBeltSpeed(-speed);}),
            Commands.runOnce(() -> {this.setSourceSpeed(speed);}),
            Commands.runOnce(() -> {this.setAmpSpeed(speed);})
        );
    }

    public Command newGroundIntake(double speed) {
        return Commands.runOnce(() -> {this.setBeltSpeed(-speed);})
            .until(() -> m_currentPhase == Phase.PICKUP)
            .andThen(Commands.runOnce(() -> {this.setBeltSpeed(0);
        }));
    }

    public Command newerGroundIntake(double speed) {
        return Commands.runOnce(() -> {this.setBeltSpeed(-speed);})
            .until(() -> m_currentPhase == Phase.PICKUP)
            .andThen(Commands.runOnce(() -> {this.setBeltSpeed(-speed * 0.75);}))
            .until(() -> m_currentPhase == Phase.LOADED)
            .andThen(Commands.runOnce(() -> {this.setBeltSpeed(0);})
        );
    }

    public Command stopMechanism() {
        return parallel(
            Commands.runOnce(() -> {this.setBeltSpeed(0);}),
            Commands.runOnce(() -> {this.setSourceSpeed(0);}),
            Commands.runOnce(() -> {this.setAmpSpeed(0);})
        );
    }

    public void periodic() {
        this.updatePhase();
    }
}