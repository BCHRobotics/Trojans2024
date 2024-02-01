package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPositions;
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
    //private final RelativeEncoder m_rightEncoder;

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
            ElevatorConstants.kSVolts, ElevatorConstants.kGVolts, ElevatorConstants.kVVolts);
   
    /** Creates a new Mechanism. */
    public Elevator() {
        super(m_PIDController, 0);
        
        m_leftMotor = new CANSparkMax(ElevatorConstants.kLeftElevatorMotorCanId, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ElevatorConstants.kRightElevatorMotorCanId, MotorType.kBrushless);

        m_leftEncoder = m_leftMotor.getEncoder();
        //m_rightEncoder = m_rightMotor.getEncoder();

        this.m_leftMotor.restoreFactoryDefaults();
        this.m_rightMotor.restoreFactoryDefaults();

        this.m_leftMotor.setIdleMode(IdleMode.kBrake);
        this.m_rightMotor.setIdleMode(IdleMode.kBrake);

        this.m_leftMotor.setSmartCurrentLimit(60, 20);
        this.m_rightMotor.setSmartCurrentLimit(60, 20);

        this.m_rightMotor.follow(m_leftMotor);

        this.m_leftMotor.setInverted(false);
        this.m_rightMotor.setInverted(false);

        this.m_leftMotor.setOpenLoopRampRate(0.05);
        this.m_rightMotor.setOpenLoopRampRate(0.05);

        this.m_leftMotor.enableVoltageCompensation(12);
        this.m_rightMotor.enableVoltageCompensation(12);

        this.m_leftEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorPositionConversionFactor);
        //this.m_rightEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorPositionConversionFactor);

        setGoal(0);
    }

    //TODO: choose between TOP or BOTTOM position for encoder reset
    //TODO: choose what to do on default
    public Command moveToPosition(ElevatorConstants.ElevatorPositions position) {
        switch (position) {
            case TOP:
                return null;
            case SOURCE:
                return Commands.runOnce(() -> this.setGoal(8));
            case AMP:
                return Commands.runOnce(() -> this.setGoal(6));
            case TRAVEL:
                return Commands.runOnce(() -> this.setGoal(4));
            case INTAKE:
                return Commands.runOnce(() -> this.setGoal(2));
            case BOTTOM:
                return null;
            default:
                return null;
        }
    }

    public Command stopElevatorCommand() {
        return parallel (
            Commands.runOnce(() -> this.cancelAllElevatorCommands()),
            Commands.runOnce(() -> this.stopElevator()));
    }

    private double getLeftMotorSpeed() {
        return this.m_leftMotor.get();
    }

    private void setLeftMotorSpeed(double speed) {
        this.m_leftMotor.setVoltage(speed);
    }

    private void stopElevator() {
        this.setLeftMotorSpeed(0);
  //      this.setRightMotorSpeed(0);
    }

    private void updateLimit() {
        this.m_elevatorLimit.updateLimit();
        this.m_currentLimitSwitch = this.m_elevatorLimit.getLimit();
    }

    private boolean checkLimit(ElevatorLimit limit) {
        return this.m_currentLimitSwitch == limit;
    }

    private void cancelAllElevatorCommands() {
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.TOP));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.SOURCE));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.AMP));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.TRAVEL));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.INTAKE));
        CommandScheduler.getInstance().cancel(moveToPosition(ElevatorPositions.BOTTOM));
    }

    public void periodic() {
        this.updateLimit();
        SmartDashboard.putString("Elevator Limit: ", this.m_currentLimitSwitch.name());
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

        if (this.checkLimit(ElevatorLimit.TOP) && output > 0) {
            this.stopElevator();
            cancelAllElevatorCommands();
            m_leftEncoder.setPosition(10); //TODO: double check top measurements
      //      m_rightEncoder.setPosition(10);
            System.out.println("Top Limit Hit");

        } else if (this.checkLimit(ElevatorLimit.BOTTOM) && output < 0) {
            this.stopElevator();
            cancelAllElevatorCommands();
            m_leftEncoder.setPosition(0);
     //       m_rightEncoder.setPosition(0);
            System.out.println("Bottom Limit Hit");

        } else {
            // Add the feedforward to the PID output to get the motor output
            this.setLeftMotorSpeed(output + feedforward);
   //         this.setRightMotorSpeed(output + feedforward);
        }
    }

    @Override
    protected double getMeasurement() {
        return m_leftEncoder.getPosition();
    }
}