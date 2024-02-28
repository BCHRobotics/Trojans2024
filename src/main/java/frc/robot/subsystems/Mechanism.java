// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.ElevatorConstants.kElevatorPositions;
import frc.utils.BeamBreak;
import frc.utils.LEDs;
import frc.utils.BeamBreak.Phase;

public class Mechanism extends SubsystemBase{
    // The beam-break sensor that detects where a note is in the mechanism
    private final BeamBreak m_beamBreak = new BeamBreak();

    private Elevator m_elevator;

    private LEDs m_LEDs;

    // The phase of the beam-break sensor
    private Phase m_currentPhase = Phase.NONE;

    private final CANSparkMax m_bottomBeltMotor = new CANSparkMax(MechanismConstants.kBottomBeltMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_topBeltMotor = new CANSparkMax(MechanismConstants.kTopBeltMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_sourceMotor = new CANSparkMax(MechanismConstants.kSourceMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_ampMotor = new CANSparkMax(MechanismConstants.kAmpMotorCanId, MotorType.kBrushless);

    /** Creates a new Mechanism. */
    public Mechanism() {
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
    private void setBeltSpeed(double speed) {
        this.m_bottomBeltMotor.setVoltage(speed);
        this.m_topBeltMotor.setVoltage(speed);
    }

    /**
     * Sets the speed of the source motor
     * @param speed the speed in volts [0 --> 12]
     */
    private void setSourceSpeed(double speed) {
        this.m_sourceMotor.setVoltage(speed);
    }
    
    /**
     * Sets the speed of the amp motor
     * @param speed the speed in volts [0 --> 12]
     */
    private void setAmpSpeed(double speed) {
        this.m_ampMotor.setVoltage(speed);
    }

    /**
     * A method to check if the phase changed
     * @param phase the phase that is checked
     */
    private boolean checkState(Phase phase) {
        if (this.m_currentPhase == Phase.NONE) {
            this.m_LEDs.setLEDs(true);
        }
        else {
            this.m_LEDs.setLEDs(false);
        }

        return m_currentPhase == phase;
    }

    public Command groundIntake(double speed) {
        return this.startEnd(
            () -> {
                this.setBeltSpeed(-speed);
                this.setSourceSpeed(speed);
                this.setAmpSpeed(speed);
            },

            () -> {
                this.setBeltSpeed(-speed * 0.75);
                this.setSourceSpeed(speed * 0.75);
                this.setAmpSpeed(speed * 0.75);
            })
            .until(() -> this.checkState(Phase.GROUND_PICKUP))
            .andThen(startEnd(
                () -> {
                this.setBeltSpeed(-speed * 0.75);
                this.setSourceSpeed(speed * 0.75);
                this.setAmpSpeed(speed * 0.75);
            },
            () -> {
                this.setBeltSpeed(0.0);
                this.setSourceSpeed(0.0);
                this.setAmpSpeed(0.0);
            }).until(() -> this.checkState(Phase.LOADED))
        //    .andThen(() -> this.m_LEDs.setLEDs(true))
        );
      }

    public Command sourceIntake(double speed) {
        return this.startEnd(
            () -> {
                this.setBeltSpeed(speed);
                this.setSourceSpeed(-speed);
                this.setAmpSpeed(-speed);
            },

            () -> {
                this.setBeltSpeed(speed * 0.75);
                this.setSourceSpeed(-speed * 0.75);
                this.setAmpSpeed(-speed * 0.75);
            })
            .until(() -> this.checkState(Phase.SOURCE_INTAKE))
            .andThen(startEnd(
                () -> {
                this.setBeltSpeed(speed * 0.75);
                this.setSourceSpeed(-speed * 0.75);
                this.setAmpSpeed(-speed * 0.75);
            },
            () -> {
                this.setBeltSpeed(0.0);
                this.setSourceSpeed(0.0);
                this.setAmpSpeed(0.0);
            })
            .until(() -> this.checkState(Phase.LOADED))
           // .andThen(() -> this.m_LEDs.setLEDs(true))
            .andThen(() -> this.m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE))
        );
    }

    public Command scoreAmp(double speed) {
        return this.startEnd(
            () -> {
                this.setBeltSpeed(-speed);
                this.setSourceSpeed(speed);
                this.setAmpSpeed(-speed * 0.7);
            },

            () -> {
                this.setBeltSpeed(-speed);
                this.setSourceSpeed(speed);
                this.setAmpSpeed(-speed * 0.7);
            }
        )
        .until(() -> this.checkState(Phase.NONE)) //before it was Phase.SOURCE_INTAKE
        .andThen(
            this.runOnce(
                () -> {
                    this.setBeltSpeed(0);
                    this.setSourceSpeed(0);
                    this.setAmpSpeed(0);
                }
            )
            .beforeStarting(new WaitCommand(1))
            //.andThen(() -> this.m_LEDs.setLEDs(false))
            .andThen(() -> this.m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE))
        );
    }

    public Command scoreSpeaker(double speed) {
        return this.runOnce(
            () -> {
                this.setSourceSpeed(speed);
                this.setAmpSpeed(speed);
            }
        )
        .andThen(
            this.runOnce(
                () -> {
                    this.setBeltSpeed(-speed);
                }
            )
            .beforeStarting(new WaitCommand(1))
        )
        .until(() -> this.checkState(Phase.NONE))
        .andThen(() -> this.m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE))
        .andThen(
            this.runOnce(
                () -> {
                    this.setBeltSpeed(0);
                    this.setSourceSpeed(0);
                    this.setAmpSpeed(0);
                }
            )
           // .andThen(() -> this.m_LEDs.setLEDs(false))
        );
    }

    public Command stopMechanism() {
        return runOnce(() -> {
          this.setBeltSpeed(0);
          this.setSourceSpeed(0);
          this.setAmpSpeed(0);
        });
    }

    public void periodic() {
        this.updatePhase();
        SmartDashboard.putString("Current Phase: ", this.m_currentPhase.name());
    }
}