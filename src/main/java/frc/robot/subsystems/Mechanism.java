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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ElevatorConstants.kElevatorPositions;
import frc.utils.BeamBreak;
import frc.utils.LEDs;
import frc.utils.BeamBreak.Phase;

public class Mechanism extends SubsystemBase{
    // The beam-break sensor that detects where a note is in the mechanism
    private final BeamBreak m_beamBreak = new BeamBreak();

   // private Elevator m_elevator;

    private LEDs m_LEDs = new LEDs();

    // The phase of the beam-break sensor
    private Phase m_currentPhase = Phase.NONE;

    private int requestIntakeType;

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

        requestIntakeType = 0;
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
    public boolean checkState(Phase phase) {
        if (this.m_currentPhase != Phase.NONE) {
            System.out.println("here");
        }

        return m_currentPhase == phase;
    }

    /**
     * A function for requesting ground/source intake using LEDs
     * @param intakeType The requested intake type (1 is ground, 2 is source, 0 is nothing)
     */
    public void requestIntake(int intakeType) {
        // Set the requested intake based on the input
        if (requestIntakeType == intakeType) { // If you request the same intake twice the lights turn off
            requestIntakeType = 0;
        }
        else {
            requestIntakeType = intakeType;
        }

        // Set the LED color to the requested intake
        if (this.m_currentPhase != Phase.NONE) {
            requestIntakeType = 0;
        }
        else if (requestIntakeType == 2) {
            this.powerLEDs("cyan");
        }
        else if (requestIntakeType == 1) {
            this.powerLEDs("purple");
        }
        else {
            this.powerLEDs("off");
        }
    }

    /**
     * ground intake command
     * @param speed the speed to run the ground intake at in volts [0 --> 12]
     * @return
     */
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
            }).until(() -> this.checkState(Phase.LOADED)))
            .andThen(confirmIntake());
      }

    /**
     * source intake command
     * @param speed the speed to run the source intake at in volts [0 --> 12]
     * @return
     */
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
           // .andThen(() -> this.m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE))
        ).andThen(confirmIntake());
    }

    /**
     * score amp command
     * @param speed the speed to run the amp at in volts [0 --> 12]
     * @return
     */
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
           // .andThen(() -> this.m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE))
        ).andThen(lightsOff());
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
            this.runOnce(
                () -> {
                    this.setBeltSpeed(-speed);
                }
            )
            .beforeStarting(new WaitCommand(1))
        )
        .until(() -> this.checkState(Phase.NONE))
       // .andThen(() -> this.m_elevator.moveToPositionCommand(kElevatorPositions.INTAKE))
        .andThen(
            this.runOnce(
                () -> {
                    this.setBeltSpeed(0);
                    this.setSourceSpeed(0);
                    this.setAmpSpeed(0);
                }
            )
        ).andThen(lightsOff());
    }

    public Command stopMechanism() {
        return runOnce(() -> {
          this.setBeltSpeed(0);
          this.setSourceSpeed(0);
          this.setAmpSpeed(0);
        });
    }

    /*
     * A command for the rainbow-light-thing
     */
    public Command lightShow() {
        return Commands.sequence(
            this.runOnce(() -> this.powerLEDs("red")),
            new WaitCommand(0.25),
            this.runOnce(() -> this.powerLEDs("blue")),
            new WaitCommand(0.35),
            this.runOnce(() -> this.powerLEDs("green")),
            new WaitCommand(0.25),
            this.runOnce(() -> this.powerLEDs("yellow")),
            new WaitCommand(0.35),
            this.runOnce(() -> this.powerLEDs("purple")),
            new WaitCommand(0.25),
            this.runOnce(() -> this.powerLEDs("cyan")),
            new WaitCommand(0.35)
        ).repeatedly();
    }

    /*
     * A command for confirming an intake
     */
    public Command confirmIntake() {
        return Commands.sequence(
            this.runOnce(() -> this.powerLEDs("green")),
            new WaitCommand(0.2),
            this.runOnce(() -> this.powerLEDs("off")),
            new WaitCommand(0.2),
            this.runOnce(() -> this.powerLEDs("green")),
            new WaitCommand(0.2),
            this.runOnce(() -> this.powerLEDs("off")),
            new WaitCommand(0.2),
            this.runOnce(() -> this.powerLEDs("green"))
        );
    }

    public Command lightsOff() {
        return Commands.sequence(
            this.runOnce(() -> this.powerLEDs("off"))
        );
    }

    /**
     * A function for changing the color of the LEDs using a string
     * @param colour the desired color name
     */
    public void powerLEDs(String colour) {
        colour = colour.toLowerCase();

        switch (colour) {
            case "red" -> this.m_LEDs.setLEDs(LEDConstants.kLEDRed);
            case "green" -> this.m_LEDs.setLEDs(LEDConstants.kLEDGreen);
            case "blue" -> this.m_LEDs.setLEDs(LEDConstants.kLEDBlue);
            case "yellow" -> this.m_LEDs.setLEDs(LEDConstants.kLEDYellow);
            case "purple" -> this.m_LEDs.setLEDs(LEDConstants.kLEDPurple);
            case "cyan" -> this.m_LEDs.setLEDs(LEDConstants.kLEDCyan);
            case "white" -> this.m_LEDs.setLEDs(LEDConstants.kLEDWhite);
            case "off" -> this.m_LEDs.setLEDs(LEDConstants.kLEDOff);  
            default -> System.out.println("That LED Color Doesn't Exist!");
        }
    }

    public void periodic() {
        this.updatePhase();
        SmartDashboard.putString("Current Phase: ", this.m_currentPhase.name());
    }
}