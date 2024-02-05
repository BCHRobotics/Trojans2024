package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
import frc.utils.NewProfiledPIDController;

public class Elevator extends SubsystemBase {

    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    private final RelativeEncoder m_leftEncoder;

    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;

    public String kEnable;
    public String kDisable;

    private static final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxSpeedMetersPerSecond,
                ElevatorConstants.kMaxAccelerationMetersPerSecondSquared);

    private static NewProfiledPIDController m_controller = new NewProfiledPIDController(
            ElevatorConstants.kPThetaController,
            0,
            ElevatorConstants.kDThetaController,
            m_constraints);

      private final ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.kSVolts, ElevatorConstants.kGVolts, ElevatorConstants.kVVolts);
   
    double totalSpeed = 0;

    SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (voltage) -> this.runVolts(voltage),
      null,
      this
    )
  );

    /** Creates a new Mechanism. */
    public Elevator() {
        
        m_leftMotor = new CANSparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);

        m_leftEncoder = m_leftMotor.getEncoder();

        this.m_leftMotor.restoreFactoryDefaults();
        this.m_rightMotor.restoreFactoryDefaults();

        this.m_leftMotor.setIdleMode(IdleMode.kBrake);
        this.m_rightMotor.setIdleMode(IdleMode.kBrake);

        this.m_leftMotor.setSmartCurrentLimit(60, 20);
        this.m_rightMotor.setSmartCurrentLimit(60, 20);

        m_forwardLimit = m_leftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_reverseLimit = m_leftMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        m_forwardLimit.enableLimitSwitch(false);
        m_reverseLimit.enableLimitSwitch(false);

        // TODO: Undo this when actually want the right motor to work Tim
        // this.m_rightMotor.follow(m_leftMotor);

        this.m_leftMotor.setInverted(false);
        this.m_rightMotor.setInverted(false);

        this.m_leftMotor.setOpenLoopRampRate(0.05);
        this.m_rightMotor.setOpenLoopRampRate(0.05);

        this.m_leftMotor.enableVoltageCompensation(12);
        this.m_rightMotor.enableVoltageCompensation(12);

        this.m_leftEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorPositionConversionFactor);

        m_leftEncoder.setPosition(0);
        m_controller.setGoal(0);
        
    }

    public void runVolts(Measure<Voltage> volts) {
        m_leftMotor.setVoltage(volts.in(Volts));
    }

    //TODO: choose between TOP or BOTTOM position for encoder reset
    //TODO: choose what to do on default
    public Command moveToPosition(ElevatorConstants.ElevatorPositions position) {
        switch (position) {
            case TOP:
                return null;
            case SOURCE:
                return Commands.runOnce(() -> m_controller.setGoal(14));
            case AMP:
                return Commands.runOnce(() -> m_controller.setGoal(6));
            case TRAVEL:
                return Commands.runOnce(() -> m_controller.setGoal(4));
            case INTAKE:
                return Commands.runOnce(() -> m_controller.setGoal(0));
            case BOTTOM:
                return null;
            default:
                return null;
        }
    }

    public Command stopElevatorCommand() {
        return Commands.runOnce(() -> this.cancelAllElevatorCommands());
    }

    private void setLeftMotorSpeed(double speed) {
        this.m_leftMotor.setVoltage(speed);
    }

    private boolean checkLimit(SparkLimitSwitch limitSwitch) {
        return limitSwitch.isPressed();
    }

    private void cancelAllElevatorCommands() {
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.TOP));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.SOURCE));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.AMP));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.TRAVEL));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.INTAKE));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.BOTTOM));
    }

    @Override
    public void periodic() {
        calculateSpeed();
        putToDashboard();
    }

    private Command resetLimit(SparkLimitSwitch limitSwitch, double goal, double motorSpeed) {
        return Commands.runOnce(() -> this.setLeftMotorSpeed(0))
                .andThen(() -> 
                    Commands.runOnce(() -> this.setLeftMotorSpeed(motorSpeed)))
                    .until(() -> this.checkLimit(limitSwitch))
                    .andThen(() -> m_controller.setGoal(goal));
    }

    public void calculateSpeed() {
        if (this.checkLimit(m_forwardLimit)) {
            System.out.println("Top Limit Hit");
            cancelAllElevatorCommands();
            resetLimit(m_forwardLimit, 8, -0.2);
            m_controller.forceAtGoal();
    
        } else if (this.checkLimit(m_reverseLimit)) {
            System.out.println("Bottom Limit Hit");
            cancelAllElevatorCommands();
            resetLimit(m_reverseLimit, 2, 0.2);
            m_controller.forceAtGoal();

        } else {
            totalSpeed = m_controller.calculate(m_leftEncoder.getPosition()) 
                + m_feedforward.calculate(m_controller.getSetpoint().velocity);
            setLeftMotorSpeed(totalSpeed);
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    private void putToDashboard() {
        SmartDashboard.putNumber("Motor Speed: ", totalSpeed);
        SmartDashboard.putNumber("Encoder Position: ", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Position Tolerence: ", m_controller.getPositionTolerance());
        SmartDashboard.putNumber("Position Error: ", m_controller.getPositionError());
        SmartDashboard.putNumber("Velocity Tolerence: ", m_controller.getVelocityTolerance());
        SmartDashboard.putNumber("Velocity Error: ", m_controller.getVelocityError());
        SmartDashboard.putBoolean("At goal: ", m_controller.atGoal());
        SmartDashboard.putBoolean("At setpoint: ", m_controller.atSetpoint());
        SmartDashboard.putBoolean("Top limit switch hit: ", m_forwardLimit.isPressed());
        SmartDashboard.putBoolean("Bottom limit switch hit: ", m_reverseLimit.isPressed());
    }
}