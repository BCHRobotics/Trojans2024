// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.commands.elevator.MoveToPosition;
import frc.utils.devices.BeamBreak;
import frc.utils.devices.LEDs;
import frc.utils.devices.BeamBreak.Phase;
import frc.robot.commands.mechanism.led.ConfirmIntake;


public class Mechanism extends SubsystemBase{
    private static Mechanism instance = null;
    
    // The beam-break sensor that detects where a note is in the mechanism
    private final BeamBreak m_beamBreak = new BeamBreak();
    private LEDs m_LEDs = new LEDs();
    private Elevator m_elevator;

    // The phase of the beam-break sensor
    private Phase m_currentPhase = Phase.NONE;

    private final CANSparkMax m_bottomBeltMotor = new CANSparkMax(MechanismConstants.kBottomBeltMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_topBeltMotor = new CANSparkMax(MechanismConstants.kTopBeltMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_sourceMotor = new CANSparkMax(MechanismConstants.kSourceMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_ampMotor = new CANSparkMax(MechanismConstants.kAmpMotorCanId, MotorType.kBrushless);

    /** Creates a new Mechanism. */
    public Mechanism() {
        m_elevator = Elevator.getInstance();

        this.m_bottomBeltMotor.restoreFactoryDefaults();
        this.m_topBeltMotor.restoreFactoryDefaults();
        this.m_sourceMotor.restoreFactoryDefaults();
        this.m_ampMotor.restoreFactoryDefaults();

        this.m_bottomBeltMotor.setIdleMode(IdleMode.kBrake);
        this.m_topBeltMotor.setIdleMode(IdleMode.kBrake);
        this.m_sourceMotor.setIdleMode(IdleMode.kBrake);
        this.m_ampMotor.setIdleMode(IdleMode.kBrake);

        this.m_bottomBeltMotor.setSmartCurrentLimit(40, 20); //keep 40 for neo550
        this.m_topBeltMotor.setSmartCurrentLimit(40, 20); //keep 40 for neo550
        this.m_sourceMotor.setSmartCurrentLimit(60, 20);
        this.m_ampMotor.setSmartCurrentLimit(60, 20);

        this.m_bottomBeltMotor.setInverted(true);
        this.m_topBeltMotor.setInverted(true);
        this.m_sourceMotor.setInverted(false);
        this.m_ampMotor.setInverted(false);

        this.m_bottomBeltMotor.setOpenLoopRampRate(0.05);
        this.m_topBeltMotor.setOpenLoopRampRate(0.05);
        this.m_sourceMotor.setOpenLoopRampRate(0.05);
        this.m_ampMotor.setOpenLoopRampRate(0.05);

        this.m_bottomBeltMotor.enableVoltageCompensation(12);
        this.m_topBeltMotor.enableVoltageCompensation(12);
        this.m_sourceMotor.enableVoltageCompensation(12);
        this.m_ampMotor.enableVoltageCompensation(12);

        this.powerLEDs(LEDColor.OFF);
    }

    /**
     * Gets the instance of the mechanism
     * @return the instance of the mechanism
     */
    public static Mechanism getInstance() {
        if (instance == null) {
            instance = new Mechanism();
        }
        return instance;
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
    public void setBeltSpeed(double speed) {
        this.m_bottomBeltMotor.setVoltage(speed);
        this.m_topBeltMotor.setVoltage(speed);
    }

    /**
     * Sets the speed of the source motor
     * @param speed the speed in volts [0 --> 12]
     */
    public void setSourceSpeed(double speed) {
        this.m_sourceMotor.setVoltage(speed);
    }
    
    /**
     * Sets the speed of the amp motor
     * @param speed the speed in volts [0 --> 12]
     */
    public void setAmpSpeed(double speed) {
        this.m_ampMotor.setVoltage(speed);
    }

    public void stopMotors() {
        this.setBeltSpeed(0);
        this.setSourceSpeed(0);
        this.setAmpSpeed(0);
    }

    /**
     * A method to check if the phase changed
     * @param phase the phase that is checked
     */
    public boolean checkState(Phase phase) {
        return m_currentPhase == phase;
    }

    public Phase getPhase() {
        return m_currentPhase;
    }

    public LEDColor getColor() {
        return m_LEDs.getLEDS();
    }


    /**
     * score speaker command
     * @param speed the speed to run the speaker at in volts [0 --> 12]
     * @return
     */
    public Command scoreSpeaker(double speed) {
        return this.runOnce(
            () -> {
                this.setSourceSpeed(speed);
                this.setAmpSpeed(speed);
            }
        )
        .andThen(
            this.startEnd(
                () -> {
                    this.setBeltSpeed(-speed);
                },
                () -> {
                    this.setBeltSpeed(0);
                }
            )
            .beforeStarting(new WaitCommand(0.5))
        )
        .until(() -> this.checkState(Phase.NONE))
        .andThen(new MoveToPosition(m_elevator, ElevatorPositions.INTAKE))
        .beforeStarting(new WaitCommand(0.1))
        .andThen(
            this.runOnce(
                () -> {
                    this.setSourceSpeed(0);
                    this.setAmpSpeed(0);
                }
            )
        ).andThen(() -> this.powerLEDs(LEDColor.OFF));
    }


    /**
     * A function for changing the color of the LEDs using a string
     * @param colour the desired color name
     */
    public void powerLEDs(LEDColor color) {
        this.m_LEDs.setLEDs(color);
    }

    public void periodic() {
        this.updatePhase();
        SmartDashboard.putString("Current Phase: ", getPhase().name());
        SmartDashboard.putString("Current LEDColor: ", getColor().name());
    }

}