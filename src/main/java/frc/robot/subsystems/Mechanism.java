// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private Phase m_currentPhase = Phase.NONE;

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
     * Gets belt speed in percent output [-1 --> 1]
     */
    private double getBeltSpeed() {
        return this.m_beltMotor.get();
    }

    /**
     * Sets source intake speed in percent output [-1 --> 1]
     */
    private void setSourceSpeed(double speed) {
        this.m_sourceMotor.set(speed);
    }

    /**
     * Gets source intake speed in percent output [-1 --> 1]
     */
    private double getSourceSpeed() {
        return this.m_sourceMotor.get();
    }
    
    /**
     * Sets amp motor speed in percent output [-1 --> 1]
     */
    private void setAmpSpeed(double speed) {
        this.m_ampMotor.set(speed);
    }

    /**
     * Gets amp motor speed in percent output [-1 --> 1]
     */
    private double getAmpSpeed() {
        return this.m_ampMotor.get();
    }

    public Command scoreAmp(double speed) {
        return parallel(
            Commands.runOnce(() -> {this.setBeltSpeed(-speed);}),
            Commands.runOnce(() -> {this.setSourceSpeed(speed);}),
            Commands.runOnce(() -> {this.setAmpSpeed(-speed);})
        );
    }

    public Command sourceIntake(double speed) {
        return parallel(
            Commands.runOnce(() -> {this.setBeltSpeed(speed);}),
            Commands.runOnce(() -> {this.setSourceSpeed(-speed);}),
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
        return Commands.runOnce(() -> {this.setBeltSpeed(-speed);})             //TODO: try removing the Commands.
            .until(() -> checkState(Phase.PICKUP))                              //TODO: try checkState function
            .andThen(                                                           //TODO: try removing {} from the lambda
                Commands.runOnce(() -> {this.setBeltSpeed(-speed * 0.75);}))
                .until(() -> m_currentPhase == Phase.LOADED)
                .andThen(
                    Commands.runOnce(() -> {this.setBeltSpeed(0);})
                    .until(() -> this.getBeltSpeed() == 0));
    }

    public Command newerGroundIntake() {
        return startEnd(() -> this.setBeltSpeed(0.5), () -> this.setBeltSpeed(0.0))
            .until(() -> checkState(Phase.PICKUP))
            .andThen(
                () -> System.out.println("finished!"));
    }

    public Command evenNewerGroundIntake(double speed) {
        return Commands.runOnce(() -> this.setBeltSpeed(-speed))
            .until(() -> m_currentPhase == Phase.PICKUP)               
            .andThen(
                Commands.runOnce(() -> this.setBeltSpeed(-speed * 0.75)))
                .until(() -> m_currentPhase == Phase.LOADED)
                .andThen(
                    Commands.runOnce(() -> this.setBeltSpeed(0))
                    .until(() -> this.getBeltSpeed() == 0));
    }

    private boolean checkState(Phase phase) {
        return m_currentPhase == phase;
    }

    public Command newSourceIntake(double speed) {
        return parallel(
            Commands.runOnce(() -> {this.setSourceSpeed(-speed);})
            .until(() -> m_currentPhase == Phase.SHOOT)
            .andThen(
                Commands.runOnce(() -> {this.setSourceSpeed(-speed * 0.75);}))
                .until(() -> m_currentPhase == Phase.LOADED)
                .andThen(
                    Commands.runOnce(() -> {this.setSourceSpeed(0);})
                    .until(() -> this.getSourceSpeed() == 0)),

            Commands.runOnce(() -> {this.setBeltSpeed(speed);})
            .until(() -> m_currentPhase == Phase.SHOOT)
            .andThen(
                Commands.runOnce(() -> {this.setBeltSpeed(speed * 0.75);}))
                .until(() -> m_currentPhase == Phase.LOADED)
                .andThen(
                    Commands.runOnce(() -> {this.setBeltSpeed(0);})
                    .until(() -> this.getBeltSpeed() == 0))
        );
    }

    public Command newScoreAmp(double speed) {
        return parallel(
            Commands.runOnce(() -> {this.setAmpSpeed(-speed);}),
            Commands.runOnce(() -> {this.setSourceSpeed(speed);})
        )
        .until(() -> this.getAmpSpeed() == -speed && this.getSourceSpeed() == -speed)
        .andThen(
            Commands.runOnce(() -> {this.setBeltSpeed(speed);})
            .until(() -> m_currentPhase == Phase.SHOOT))
            .withTimeout(0.5)
            .andThen(
                parallel(
                    Commands.runOnce(() -> {this.setBeltSpeed(0);})
                    .until(() -> this.getBeltSpeed() == 0)),

                    Commands.runOnce(() -> {this.setAmpSpeed(0);})
                    .until(() -> this.getAmpSpeed() == 0),
                    
                    Commands.runOnce(() -> {this.setSourceSpeed(0);})
                    .until(() -> this.getSourceSpeed() == 0));
    }

    public Command stopMechanism() {
        return parallel(
            Commands.runOnce(() -> {this.setBeltSpeed(0);}),
            Commands.runOnce(() -> {this.setSourceSpeed(0);}),
            Commands.runOnce(() -> {this.setAmpSpeed(0);})
        )
        .until(() -> this.getBeltSpeed() == 0 && this.getSourceSpeed() == 0 && this.getAmpSpeed() == 0);
    }

    public void periodic() {
        this.updatePhase();
        SmartDashboard.putString("Current Phase: ", this.m_currentPhase.name());
    }
}