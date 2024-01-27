// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mechanism extends SubsystemBase{
    private final CANSparkMax m_topBeltMotor = new CANSparkMax(20, MotorType.kBrushless);
    private final CANSparkMax m_bottomBeltMotor = new CANSparkMax(21, MotorType.kBrushless);
    private final CANSparkMax m_sourceMotor = new CANSparkMax(22, MotorType.kBrushless);
    private final CANSparkMax m_ampMotor = new CANSparkMax(22, MotorType.kBrushless);

    /** Creates a new Mechanism. */
    public Mechanism() {
        this.m_topBeltMotor.restoreFactoryDefaults();
        this.m_bottomBeltMotor.restoreFactoryDefaults();
        this.m_sourceMotor.restoreFactoryDefaults();
        this.m_ampMotor.restoreFactoryDefaults();

        this.m_topBeltMotor.setIdleMode(IdleMode.kBrake);
        this.m_bottomBeltMotor.setIdleMode(IdleMode.kBrake);
        this.m_sourceMotor.setIdleMode(IdleMode.kBrake);
        this.m_ampMotor.setIdleMode(IdleMode.kBrake);

        this.m_topBeltMotor.setSmartCurrentLimit(60, 20);
        this.m_bottomBeltMotor.setSmartCurrentLimit(60, 20);
        this.m_sourceMotor.setSmartCurrentLimit(60, 20);
        this.m_ampMotor.setSmartCurrentLimit(60, 20);

        this.m_topBeltMotor.setInverted(false);
        this.m_bottomBeltMotor.setInverted(true);
        this.m_sourceMotor.setInverted(true);
        this.m_ampMotor.setInverted(false);

        this.m_topBeltMotor.setOpenLoopRampRate(0.05);
        this.m_bottomBeltMotor.setOpenLoopRampRate(0.05);
        this.m_sourceMotor.setOpenLoopRampRate(0.05);
        this.m_ampMotor.setOpenLoopRampRate(0.05);

        this.m_topBeltMotor.enableVoltageCompensation(12);
        this.m_bottomBeltMotor.enableVoltageCompensation(12);
        this.m_sourceMotor.enableVoltageCompensation(12);
        this.m_ampMotor.enableVoltageCompensation(12);
    }

    
    /**
     * Sets belt speed in percent output [-1 --> 1]
     */
    private void setBeltSpeed(double speed) {
        this.m_topBeltMotor.set(speed);
        this.m_bottomBeltMotor.set(speed);
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

    public Command groundIntake() {
        return parallel(
            run(() -> this.setBeltSpeed(0.5)),
            run(() -> this.setSourceSpeed(0.5)),
            run(() -> this.setAmpSpeed(0.5)));
    }

    public Command sourceIntake() {
        return parallel(
            run(() -> this.setBeltSpeed(0.5)),
            run(() -> this.setSourceSpeed(0.5)),
            run(() -> this.setAmpSpeed(0.5)));
    }

    public void stopMechanism() {
        this.setBeltSpeed(0);
        this.setSourceSpeed(0);
        this.setAmpSpeed(0);
    }

    public void periodic() {}
}