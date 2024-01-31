package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.ElevatorLimits;
import frc.utils.ElevatorLimits.ElevatorLimit;

public class Elevator extends SubsystemBase{

    // The beam-break sensor that detects where a note is in the mechanism
    private final ElevatorLimits m_elevatorLimit = new ElevatorLimits(
        ElevatorConstants.kTopElevatorLimitSwitchPort, 
        ElevatorConstants.kBottomElevatorLimitSwitchPort
    );

    private ElevatorLimit m_currentLimitSwitch = ElevatorLimit.TOP;

    private final CANSparkMax m_leftMotor = new CANSparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor = new CANSparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);

    /** Creates a new Mechanism. */
    public Elevator() {
        this.m_leftMotor.restoreFactoryDefaults();
        this.m_rightMotor.restoreFactoryDefaults();


        this.m_leftMotor.setIdleMode(IdleMode.kBrake);
        this.m_rightMotor.setIdleMode(IdleMode.kBrake);

        this.m_leftMotor.setSmartCurrentLimit(60, 20);
        this.m_rightMotor.setSmartCurrentLimit(60, 20);

        this.m_leftMotor.setInverted(false);
        this.m_rightMotor.setInverted(false);

        this.m_leftMotor.setOpenLoopRampRate(0.05);
        this.m_rightMotor.setOpenLoopRampRate(0.05);

        this.m_leftMotor.enableVoltageCompensation(12);
        this.m_rightMotor.enableVoltageCompensation(12);
    }

    private double getLeftMotorSpeed() {
        return this.m_leftMotor.get();
    }

    private double getRightMotorSpeed() {
        return this.m_rightMotor.get();
    }

    private void setLeftMotorSpeed(double speed) {
        this.m_leftMotor.set(speed);
    }

    private void setRightMotorSpeed(double speed) {
        this.m_rightMotor.set(speed);
    }

    private void updateLimit() {
        this.m_elevatorLimit.updateLimit();
        this.m_currentLimitSwitch = this.m_elevatorLimit.getLimit();
    }

    private boolean checkLimit(ElevatorLimit limit) {
        return this.m_currentLimitSwitch == limit;
    }

    

    public void periodic() {
        this.updateLimit();
        SmartDashboard.putString("Elevator Limit: ", this.m_currentLimitSwitch.name());
    }
}