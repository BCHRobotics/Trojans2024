// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MechanismConstants;
import frc.utils.BeamBreak;
import frc.utils.BeamBreak.Phase;

public class Mechanism extends SubsystemBase{
    // The beam-break sensor that detects where a note is in the mechanism
    private final BeamBreak m_beamBreak = new BeamBreak();

    // The phase of the beam-break sensor
    private Phase m_currentPhase = Phase.NONE;

    private final CANSparkMax m_beltMotor = new CANSparkMax(MechanismConstants.kBeltMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_sourceMotor = new CANSparkMax(MechanismConstants.kSourceMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_ampMotor = new CANSparkMax(MechanismConstants.kAmpMotorCanId, MotorType.kBrushless);

    /** Creates a new Mechanism. */
    public Mechanism() {
        this.m_beltMotor.restoreFactoryDefaults();
        this.m_sourceMotor.restoreFactoryDefaults();
        this.m_ampMotor.restoreFactoryDefaults();

        this.m_beltMotor.setIdleMode(IdleMode.kBrake);
        this.m_sourceMotor.setIdleMode(IdleMode.kBrake);
        this.m_ampMotor.setIdleMode(IdleMode.kBrake);

        this.m_beltMotor.setSmartCurrentLimit(40, 20); //keep 40 for neo550
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
    private void setBeltSpeed(double speed) {
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
    private void setSourceSpeed(double speed) {
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
    private void setAmpSpeed(double speed) {
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
    private boolean checkState(Phase phase) {
        return m_currentPhase == phase;
    }

    /**
     * Cancels all mechanism commands
     */
    private void cancelAllMechanismCommands() {
        CommandScheduler.getInstance().cancel(this.groundIntake(getBeltSpeed()));
        CommandScheduler.getInstance().cancel(this.scoreAmp(getAmpSpeed()));
        CommandScheduler.getInstance().cancel(this.sourceIntake(getSourceSpeed()));
    }

    /**
     * A command for intaking from the ground
     * @param speed the commanded speed in voltage [0 --> 12]
     */
    public Command groundIntake(double speed) {
        return parallel(
            Commands.startEnd(() -> this.setBeltSpeed(-speed), () -> this.setBeltSpeed(-speed * 0.75)),
            Commands.startEnd(() -> this.setSourceSpeed(speed), () -> this.setSourceSpeed(speed * 0.75)),
            Commands.startEnd(() -> this.setAmpSpeed(speed), () -> this.setAmpSpeed(speed * 0.75))
        )
        .until(() -> this.checkState(Phase.GROUND_PICKUP))
        .andThen(() -> System.out.println("BOTTOM hit"))
        .andThen(
            parallel(
                Commands.startEnd(() -> this.setBeltSpeed(-speed * 0.75), () -> this.setBeltSpeed(0.0))
                .until(() -> this.checkState(Phase.LOADED))
                .andThen(() -> System.out.println("MIDDLE hit")),

                Commands.startEnd(() -> this.setSourceSpeed(-speed * 0.75), () -> this.setSourceSpeed(0.0))
                .until(() -> this.checkState(Phase.LOADED)),

                Commands.startEnd(() -> this.setAmpSpeed(-speed * 0.75), () -> this.setAmpSpeed(0.0))
                .until(() -> this.checkState(Phase.LOADED))
            )
        );
    }

    public Command newgroundIntake(double speed) {
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
            .andThen(() -> System.out.println("BOTTOM hit"))
            .andThen(startEnd(
                () -> {
                this.setBeltSpeed(-speed * 0.75);
                this.setSourceSpeed(-speed * 0.75);
                this.setAmpSpeed(-speed * 0.75);
            },
            () -> {
                this.setBeltSpeed(0.0);
                this.setSourceSpeed(0.0);
                this.setAmpSpeed(0.0);
            }).until(() -> this.checkState(Phase.LOADED))
        );
      }
 
    /**
     * A command for intaking from the source
     * @param speed the commanded speed in voltage [0 --> 12]
     */
    public Command sourceIntake(double speed) {
        return parallel (
            Commands.startEnd(() -> this.setSourceSpeed(-speed), () -> this.setSourceSpeed(-speed * 0.75)),
            Commands.startEnd(() -> this.setAmpSpeed(-speed), () -> this.setAmpSpeed(-speed * 0.75)),
            Commands.startEnd(() -> this.setBeltSpeed(speed), () -> this.setBeltSpeed(speed * 0.75))
        )
        .until(() -> this.checkState(Phase.SOURCE_INTAKE))
        .andThen(
            parallel(
                Commands.startEnd(() -> this.setSourceSpeed(-speed * 0.75), () -> this.setSourceSpeed(0)),
                Commands.startEnd(() -> this.setAmpSpeed(-speed * 0.75), () -> this.setAmpSpeed(0)),
                Commands.startEnd(() -> this.setBeltSpeed(speed * 0.75), () -> this.setBeltSpeed(0))
            )
            .until(() -> this.checkState(Phase.LOADED))
        );
    }

    public Command newsourceIntake(double speed) {
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
        );
    }

    /**
     * A command for scoring in the amp
     * @param speed the commanded speed in voltage [0 --> 12]
     */
    public Command scoreAmp(double speed) {
        return parallel(
            Commands.startEnd(() -> this.setAmpSpeed(-speed), () -> this.setAmpSpeed(-speed))
            .until(() -> this.checkState(Phase.SOURCE_INTAKE)),

            Commands.startEnd(() -> this.setSourceSpeed(speed), () -> this.setSourceSpeed(speed))
            .until(() -> this.checkState(Phase.SOURCE_INTAKE)),

            Commands.startEnd(() -> this.setBeltSpeed(-speed), () -> this.setBeltSpeed(-speed))
            .until(() -> this.checkState(Phase.SOURCE_INTAKE))
        )
        .andThen(
            parallel(
                Commands.runOnce(() -> this.setAmpSpeed(0)),
                Commands.runOnce(() -> this.setSourceSpeed(0)),
                Commands.runOnce(() -> this.setBeltSpeed(0))
            ).beforeStarting(new WaitCommand(1))
        );
    }

    public Command newscoreAmp(double speed) {
        return this.startEnd(
            () -> {
                this.setBeltSpeed(-speed);
                this.setSourceSpeed(speed);
                this.setAmpSpeed(-speed);
            },

            () -> {
                this.setBeltSpeed(-speed);
                this.setSourceSpeed(speed);
                this.setAmpSpeed(-speed);
            }
        )
        .until(() -> this.checkState(Phase.SOURCE_INTAKE))
        .andThen(
            this.runOnce(
                () -> {
                    this.setBeltSpeed(0);
                    this.setSourceSpeed(0);
                    this.setAmpSpeed(0);
                }
            )
            .beforeStarting(new WaitCommand(1))
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