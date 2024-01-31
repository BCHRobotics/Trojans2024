package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.utils.ElevatorLimits;
import frc.utils.ElevatorLimits.ElevatorLimit;

public class Elevator extends ProfiledPIDSubsystem {

    // The beam-break sensor that detects where a note is in the mechanism
    private final ElevatorLimits m_elevatorLimit = new ElevatorLimits(
        ElevatorConstants.kTopElevatorLimitSwitchPort, 
        ElevatorConstants.kBottomElevatorLimitSwitchPort
    );

    private ElevatorLimit m_currentLimitSwitch = ElevatorLimit.TOP;

    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    private static final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxSpeedMetersPerSecond,
                ElevatorConstants.kMaxAccelerationMetersPerSecondSquared);

    private static ProfiledPIDController m_PIDController = new ProfiledPIDController(
            ElevatorConstants.kPThetaController,
            0,
            0,
            m_constraints);

      private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kSVolts, ElevatorConstants.kGVolts,
          ElevatorConstants.kVVolts);
   
    /** Creates a new Mechanism. */
    public Elevator() {
        super(m_PIDController, 0);
        
        m_leftMotor = new CANSparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);

        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();

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

        this.m_leftEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorPositionConversionFactor);
        this.m_rightEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorPositionConversionFactor);

        setGoal(0);
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

    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        m_leftMotor.setVoltage(output + feedforward);
        m_rightMotor.setVoltage(output + feedforward);
    }

    @Override
    protected double getMeasurement() {
        return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
    }
}