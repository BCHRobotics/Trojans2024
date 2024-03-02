// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum and minimum capable speeds of
    // the robot, rather the allowed maximum and minimum speeds.
    public static final double kMaxSpeedMetersPerSecond = 4.1;
    public static final double kMinSpeedMetersPerSecond = 1.6;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.4; // radians per second
    public static final double kMagnitudeSlewRate = 2.8; // percent per second (1 = 100%) // 3.6
    public static final double kRotationalSlewRate = 2.6; // percent per second (1 = 100%) // 3.0

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 12; //13
    public static final int kRearLeftDrivingCanId = 10; //15
    public static final int kFrontRightDrivingCanId = 14; //17
    public static final int kRearRightDrivingCanId = 16;  //11

    public static final int kFrontLeftTurningCanId = 13; //12
    public static final int kRearLeftTurningCanId = 11;  //14
    public static final int kFrontRightTurningCanId = 15; //16
    public static final int kRearRightTurningCanId = 17;  //10

    public static final boolean kGyroReversed = true;
  }

  public static final class ElevatorConstants {
    public static final int kLeftElevatorMotorCanId = 20;  //30
    public static final int kRightElevatorMotorCanId = 21;  //31

    public static final int kTopElevatorLimitSwitchPort = 5;
    public static final int kBottomElevatorLimitSwitchPort = 6;

    public static final double kPThetaController = 10;
    public static final double kIThetaController = 0;
    public static final double kDThetaController = 0;

    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;

    public static final double kSVolts = 0.096656;
    public static final double kGVolts = 0.1647;
    public static final double kVVolts = 0.0021784;
    public static final double kAVolts = 0.00019749;

    //20:1 from gearbox to output reduction
    //18 tooth
    //1.432in
    public static final double kElevatorMotorReduction = 20;
    public static final double kElevatorWheelPitchDiameterInches = 1.432;
    public static final double kElevatorMotorCPR = 42;

    public static final double kElevatorPositionConversionFactor = 
                        (kElevatorWheelPitchDiameterInches * Math.PI) / 
                        (kElevatorMotorReduction * kElevatorMotorCPR);
      
    public enum kElevatorPositions {
      SOURCE,
      AMP,
      INTAKE
    }

    public static double[] kElevatorGoals = new double[] {
      0.39,
      0.32,
      -0.01
    };
  }
  public static final class MechanismConstants {
    public static final int kBottomBeltMotorCanId = 33;
    public static final int kTopBeltMotorCanId = 32;
    public static final int kSourceMotorCanId = 30;
    public static final int kAmpMotorCanId = 31;

    public static final int kBottomSensorChannel = 4;
    public static final int kMiddleSensorChannel = 5;
    public static final int kTopSensorChannel = 6;
  }

  public static final class LEDConstants {
    public static final int kRedLEDPort = 7;
    public static final int kGreenLEDPort = 8;
    public static final int kBlueLEDPort = 9;

    public static boolean[] kLEDRed = new boolean[] {true, false, false};
    public static boolean[] kLEDGreen = new boolean[] {false, true, false};
    public static boolean[] kLEDBlue = new boolean[] {false, false, true};
    public static boolean[] kLEDYellow = new boolean[] {true, true, false};
    public static boolean[] kLEDPurple = new boolean[] {true, false, true};
    public static boolean[] kLEDCyan = new boolean[] {false, true, true};
    public static boolean[] kLEDWhite = new boolean[] {true, true, true};
    public static boolean[] kLEDOff = new boolean[] {false, false, false};
  }
  public static final class VisionConstants{
    // Height of the camera (not used anywhere right now)
    public static final double kCameraHeight = 0.0;

    // Speed and rotation caps for vision
    public static final double kVisionSpeedLimit = 0.3;
    public static final double kVisionTurningLimit = 0.2;

    // How close to an apriltag the robot has to be before stopping (meters)
    public static final double kTagDistanceThreshold = 0.1;
    // The amount of rotational error alowed (degrees)
    public static final double kTagRotationThreshold = 7.5;

    public static final double kTagOffsetX = 0.5; // (meters)
    public static final double kTagOffsetY = 0; // (meters)

    public static final double kTagSlowdownDistance = 0.45;

    // Camera names (a and b are TEMPORARY NAMES)
    public static final String kNoteCameraName = "a";
    public static final String kTagCameraName = "b";

    // PID values for aligning to a note
    public static final double kNoteP = 0.02;
    public static final double kNoteI = 0.004;
    public static final double kNoteD = 0.0003;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kFlightstickPort = 3;
    public static final int kXBoxPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kTurnDeadband = 0.12;
    public static final double kTwistDeadband = 0.5;
    public static final boolean kFieldRelative = true;
    public static final boolean kRateLimited = true;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    //distance from robot center to furthest module
    public static final double kDriveBase = Units.inchesToMeters((Math.sqrt(Math.pow(DriveConstants.kTrackWidth, 2) 
        + Math.pow(DriveConstants.kWheelBase, 2))) / 2);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

}
