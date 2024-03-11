package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.utils.controllers.BetterPIDController;
import frc.utils.controllers.BetterProfiledPIDController;

public class Elevator extends SubsystemBase {
    private static Elevator instance = null;

    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    private final RelativeEncoder m_leftEncoder;

    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;

    private static final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxSpeedMetersPerSecond,
                ElevatorConstants.kMaxAccelerationMetersPerSecondSquared);

    private BetterProfiledPIDController m_controller = new BetterProfiledPIDController(
            ElevatorConstants.kPThetaController,
            ElevatorConstants.kIThetaController,
            ElevatorConstants.kDThetaController,
            m_constraints);

    private final ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.kSVolts, 
            ElevatorConstants.kGVolts, 
            ElevatorConstants.kVVolts, 
            ElevatorConstants.kAVolts
        );
   
    /** Creates a new Elevator. */
    public Elevator() {
        
        m_leftMotor = new CANSparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);

        m_leftEncoder = m_leftMotor.getEncoder();

        this.m_leftMotor.restoreFactoryDefaults();
        this.m_rightMotor.restoreFactoryDefaults();

        this.m_rightMotor.follow(m_leftMotor, true);

        this.m_leftMotor.setIdleMode(IdleMode.kBrake);
        this.m_rightMotor.setIdleMode(IdleMode.kBrake);

        this.m_leftMotor.setSmartCurrentLimit(60, 20);
        this.m_rightMotor.setSmartCurrentLimit(60, 20);

        m_forwardLimit = m_leftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_reverseLimit = m_leftMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        m_forwardLimit.enableLimitSwitch(true);
        m_reverseLimit.enableLimitSwitch(true);

        this.m_leftMotor.setInverted(true);

        this.m_leftMotor.setOpenLoopRampRate(0.05);

        this.m_leftMotor.enableVoltageCompensation(12);
        this.m_rightMotor.enableVoltageCompensation(12);

        this.m_leftEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorPositionConversionFactor);
        this.m_leftEncoder.setVelocityConversionFactor(ElevatorConstants.kElevatorPositionConversionFactor);

        m_leftEncoder.setPosition(0);

        m_controller.setTolerance(0.005);
        m_controller.setGoal(0);
    }

    public BetterProfiledPIDController getController() {
        return m_controller;
    }

    /**
     * Gets the instance of the elevator
     * @return the instance of the elevator
     */
    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
    
    /**
     * Sets the speed of the drive motor
     * @param speed speed in percent [0 --> 1]
     */
    private void setMotorSpeed(double speed) {
        MathUtil.clamp(speed, 0, 1);

        this.m_leftMotor.setVoltage(speed * 12);
    }

    /**
     * Calculates and sets the profiled speed of the motor
     */
    private void calculateSpeed() {
        double pidSpeed = m_controller.calculate(m_leftEncoder.getPosition());
        double ffSpeed = m_feedforward.calculate(m_controller.getSetpoint().velocity);

        setMotorSpeed(pidSpeed + ffSpeed);
    }

    public boolean limitHit() {
        return m_forwardLimit.isPressed() || m_reverseLimit.isPressed();
    }

    
    private void putToDashboard() {
        // SmartDashboard.putNumber("Total output speed", totalSpeed);
        // SmartDashboard.putNumber("Encoder Position: ", m_leftEncoder.getPosition());
        // SmartDashboard.putBoolean("At goal: ", m_controller.atGoal());
        // SmartDashboard.putBoolean("At setpoint: ", m_controller.atSetpoint());
        // SmartDashboard.putBoolean("Top limit switch hit: ", m_forwardLimit.isPressed());
        // SmartDashboard.putBoolean("Bottom limit switch hit: ", m_reverseLimit.isPressed());
    }
    
    @Override
    public void periodic() {
        this.calculateSpeed();
        this.putToDashboard();
    }
}