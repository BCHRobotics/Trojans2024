package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.kElevatorPositions;
import frc.utils.BetterProfiledPIDController;

public class Elevator extends SubsystemBase {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    private final RelativeEncoder m_leftEncoder;

    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;

    private static final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.kMaxSpeedMetersPerSecond,
                ElevatorConstants.kMaxAccelerationMetersPerSecondSquared);

    private static BetterProfiledPIDController m_controller = new BetterProfiledPIDController(
            ElevatorConstants.kPThetaController,
            0,
            ElevatorConstants.kDThetaController,
            m_constraints);

    private final ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.kSVolts, ElevatorConstants.kGVolts, ElevatorConstants.kVVolts);
   
    double totalSpeed = 0;

    private boolean forcedGoal = false;

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

        this.m_leftMotor.setInverted(false);

        this.m_leftMotor.setOpenLoopRampRate(0.05);

        this.m_leftMotor.enableVoltageCompensation(12);
        this.m_rightMotor.enableVoltageCompensation(12);

        this.m_leftEncoder.setPositionConversionFactor(ElevatorConstants.kElevatorPositionConversionFactor);

        m_leftEncoder.setPosition(0);
        m_controller.setGoal(0);
    }

    /**
     * Checks if the elevator postion is at the goal
     * @return if goal is reached
     */
    public static boolean checkAtGoal() {
        return m_controller.atGoal();
    }
    
    /**
     * Sets the speed of the drive motor
     * @param speed speed in volts [0 --> 12]
     */
    private void setLeftMotorSpeed(double speed) {
        this.m_leftMotor.setVoltage(speed);
    }

    /**
     * Checks to see if a limit switched is being pressed
     * @param limitSwitch The limit switch to check
     * @return if the limit switch is being pressed
     */
    private boolean checkLimitSwitchPress(SparkLimitSwitch limitSwitch) {
        return limitSwitch.isPressed();
    }

    /**
     * Stops the elevator and sets the goal to the current setpoint
     */
    private void limitReached() {
        System.out.println("in reached limit");
        cancelAllElevatorCommands();
        m_controller.forceAtGoal();
        forcedGoal = true;
        System.out.println("forced goal");
    }

    /**
     * Calculates and sets the profiled speed of the motor
     */
    private void calculateSpeed() {
        totalSpeed = m_controller.calculate(m_leftEncoder.getPosition())
                     + m_feedforward.calculate(m_controller.getSetpoint().velocity);
        setLeftMotorSpeed(totalSpeed);
    }

    /**
     * Checks for limits and sets the motor speed
     */
    private void setProfiledSpeed() {
        if (!forcedGoal && 
           (checkLimitSwitchPress(m_forwardLimit) || 
            checkLimitSwitchPress(m_reverseLimit))) {

            System.out.println(checkLimitSwitchPress(m_forwardLimit) 
                ? "Top Limit Hit" : "Bottom Limit Hit");

            limitReached();
        } else {
            calculateSpeed();
            if (!this.checkLimitSwitchPress(m_forwardLimit) 
             && !this.checkLimitSwitchPress(m_reverseLimit)) forcedGoal = false;
        }
    }

    /**
     * Cancels all elevator commands
     */
    private void cancelAllElevatorCommands() {
        CommandScheduler.getInstance().cancel(this.moveToPositionCommand(kElevatorPositions.SOURCE));
        CommandScheduler.getInstance().cancel(this.moveToPositionCommand(kElevatorPositions.AMP));
        CommandScheduler.getInstance().cancel(this.moveToPositionCommand(kElevatorPositions.TRAVEL));
        CommandScheduler.getInstance().cancel(this.moveToPositionCommand(kElevatorPositions.INTAKE));
    }

    /**
     * Sets the elevator positions
     * @param position the position to be set
     * @return the command to get to the position
     */
    public Command moveToPositionCommand(ElevatorConstants.kElevatorPositions position) {
        switch (position) {
            case AMP:
                return this.runOnce(() -> Elevator.m_controller.setGoal(
                    ElevatorConstants.kElevatorGoals[
                    ElevatorConstants.kElevatorPositions.AMP.ordinal()]));

            case SOURCE:
                return this.runOnce(() -> Elevator.m_controller.setGoal(
                    ElevatorConstants.kElevatorGoals[
                    ElevatorConstants.kElevatorPositions.SOURCE.ordinal()]));

            case TRAVEL:
                return this.runOnce(() -> Elevator.m_controller.setGoal(
                    ElevatorConstants.kElevatorGoals[
                    ElevatorConstants.kElevatorPositions.TRAVEL.ordinal()]));

            case INTAKE:
                return this.runOnce(() -> Elevator.m_controller.setGoal(
                    ElevatorConstants.kElevatorGoals[
                    ElevatorConstants.kElevatorPositions.INTAKE.ordinal()]));

            default:
                return null;
        }
    }

    /**
     * Stops the elevator
     * @return the command for stopping the elevator
     */
    public Command stopElevatorCommand() {
        return this.runOnce(() -> cancelAllElevatorCommands());
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
    
    @Override
    public void periodic() {
        setProfiledSpeed();
        putToDashboard();
    }
}