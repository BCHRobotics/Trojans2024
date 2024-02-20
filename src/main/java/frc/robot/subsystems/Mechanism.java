// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MechanismConstants;
import frc.robot.commands.IntakeCommands;
import frc.utils.BeamBreak;
import frc.utils.BeamBreak.Phase;

public class Mechanism extends SubsystemBase{
    private IntakeCommands m_intakeCommands;

    // The beam-break sensor that detects where a note is in the mechanism
    private final BeamBreak m_beamBreak = new BeamBreak(
        MechanismConstants.kPickupSensorChannel, 
        MechanismConstants.kLoadedSensorChannel, 
        MechanismConstants.kShootSensorChannel
    );

    // The phase of the beam-break sensor
    private Phase m_currentPhase = Phase.NONE;

    private final CANSparkMax m_beltMotor = new CANSparkMax(MechanismConstants.kBeltMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_sourceMotor = new CANSparkMax(MechanismConstants.kSourceMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_ampMotor = new CANSparkMax(MechanismConstants.kAmpMotorCanId, MotorType.kBrushless);

    /** Creates a new Mechanism. */
    protected Mechanism() {
        this.m_beltMotor.restoreFactoryDefaults();
        this.m_sourceMotor.restoreFactoryDefaults();
        this.m_ampMotor.restoreFactoryDefaults();

        this.m_beltMotor.setIdleMode(IdleMode.kBrake);
        this.m_sourceMotor.setIdleMode(IdleMode.kBrake);
        this.m_ampMotor.setIdleMode(IdleMode.kBrake);

        this.m_beltMotor.setSmartCurrentLimit(60, 20);
        this.m_sourceMotor.setSmartCurrentLimit(60, 20);
        this.m_ampMotor.setSmartCurrentLimit(60, 20);

        this.m_beltMotor.setInverted(true);
        this.m_sourceMotor.setInverted(false);
        this.m_ampMotor.setInverted(false);

        this.m_beltMotor.setOpenLoopRampRate(0.05);
        this.m_sourceMotor.setOpenLoopRampRate(0.05);
        this.m_ampMotor.setOpenLoopRampRate(0.05);

        this.m_beltMotor.enableVoltageCompensation(12);
        this.m_sourceMotor.enableVoltageCompensation(12);
        this.m_ampMotor.enableVoltageCompensation(12);
    }

    /**
     * updates the phase of the beam break sensor
     */
    private void updatePhase() {
        this.m_beamBreak.updatePhase();
        this.m_currentPhase = this.m_beamBreak.getPhase();
    }

    /**
     * Sets the speed of the belt motor
     * @param speed the speed in volts [0 --> 12]
     */
    protected void setBeltSpeed(double speed) {
        this.m_beltMotor.setVoltage(speed);
    }

    /**
     * Gets the motor voltage of the belt motor
     * @return the voltage the motor is getting
     */
    private double getBeltSpeed() {
        return this.m_beltMotor.getBusVoltage();
    }

    /**
     * Sets the speed of the source motor
     * @param speed the speed in volts [0 --> 12]
     */
    protected void setSourceSpeed(double speed) {
        this.m_sourceMotor.setVoltage(speed);
    }

    /**
     * Gets the motor voltage of the source motor
     * @return the voltage the motor is getting
     */
    private double getSourceSpeed() {
        return this.m_sourceMotor.getBusVoltage();
    }
    
    /**
     * Sets the speed of the amp motor
     * @param speed the speed in volts [0 --> 12]
     */
    protected void setAmpSpeed(double speed) {
        this.m_ampMotor.setVoltage(speed);
    }

    /**
     * Gets the motor voltage of the amp motor
     * @return the voltage the motor is getting
     */
    private double getAmpSpeed() {
        return this.m_ampMotor.getBusVoltage();
    }

    /**
     * A method to check if the phase changed
     * @param phase the phase that is checked
     */
    protected boolean checkState(Phase phase) {
        return m_currentPhase == phase;
    }

    /**
     * Cancels all mechanism commands
     */
    protected void cancelAllMechanismCommands() {
        if (m_intakeCommands != null) {
            CommandScheduler.getInstance().cancel(m_intakeCommands.groundIntake(getBeltSpeed()));
            CommandScheduler.getInstance().cancel(m_intakeCommands.scoreAmp(getAmpSpeed()));
            CommandScheduler.getInstance().cancel(m_intakeCommands.sourceIntake(getSourceSpeed()));
        }
    }

    public void periodic() {
        this.updatePhase();
        SmartDashboard.putString("Current Phase: ", this.m_currentPhase.name());
    }
}