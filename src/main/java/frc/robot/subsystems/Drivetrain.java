// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.utils.SwerveUtils;
import frc.utils.devices.Camera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  // A percentage value (0-1) for the linear speed of the robot
  private double m_maxSpeed = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private boolean m_slowMode = false;

  // If you switch the camera you have to change the name property of this
  private final Camera m_noteCamera = new Camera(VisionConstants.NOTE_CAMERA_NAME); // These names might need to be changed
  private final Camera m_tagCamera = new Camera(VisionConstants.TAG_CAMERA_NAME); // this too

  // Whether or not to try and align with a target
  private boolean isAlignmentActive = false;
  // Is true when the robot has finished a vision command
  private boolean isAlignmentSuccess = false;
  private boolean cameraMode = false;

  // The stored field position of the target apriltag
  private Pose2d targetPose;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  // Construct PhotonPoseEstimator
  //PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_tagCamera.getInstance(), robotToCam); // I'm using getInstance here as a temporary solution

  /*
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
  }
  */

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    this.initializeAuto();

    cameraMode = false;
  }

  @Override
  public void periodic() {
    // Refresh the data gathered by the camera
    m_noteCamera.refreshResult();
    m_tagCamera.refreshResult();

    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Print debug values to smartDashboard
    this.printToDashboard();

    // Update the target pose
    if (m_tagCamera.getResult().hasTargets()) {
      targetPose = m_tagCamera.getApriltagPose(getPose(), getHeading());
    }
  }

  /**
   * Checks whether the robot has finished aligning with a target
   * 
   * (ONLY USED DURING AUTO)
   * this functions as the end condition for the alignment functions
   * 
   * @return whether or not the robot has finished aligning
   */
  public boolean checkAlignment() {
    return isAlignmentSuccess; // This boolean variable is true when the robot has finished aligning (to either a tag or note)
  }

  /**
   * A function for driving to the targeted apriltag, runs periodically 
   * 
   * (ONLY USED DURING AUTO)
   */
  public void driveToTag() {
      // Apriltag alignment code
      if (isAlignmentActive && targetPose != null && cameraMode) {
        Pose2d robotPose = getPose();

        double xCommand = targetPose.getX() - robotPose.getX();
        double yCommand = targetPose.getY() - robotPose.getY();

        double tagRotation = targetPose.getRotation().getDegrees();

        if (tagRotation > 0) {
          tagRotation -= 180;
        }
        else {
          tagRotation += 180;
        }

        double rotCommand = tagRotation - getHeading();
        rotCommand *= Math.abs(rotCommand); // square the rot command for smoother rotation

        // Do not let the commanded speed above a certain value
        if (xCommand > 0) {
          xCommand = Math.min(xCommand, VisionConstants.VISION_SPEED_LIMIT);
        }
        else {
          xCommand = Math.max(xCommand, -VisionConstants.VISION_SPEED_LIMIT);
        }

        if (yCommand < 0) {
          yCommand = Math.max(yCommand, -VisionConstants.VISION_SPEED_LIMIT);
        }
        else {
          yCommand = Math.min(yCommand, VisionConstants.VISION_SPEED_LIMIT);
        }

        if (rotCommand < 0) {
          rotCommand = Math.max(rotCommand, -VisionConstants.VISION_TURN_LIMIT);
        } else {
          rotCommand = Math.min(rotCommand, VisionConstants.VISION_TURN_LIMIT);
        }

        if (Math.abs(rotCommand) < VisionConstants.APRILTAG_ROTATION_THRESHOLD && Math.abs(targetPose.getY() - robotPose.getY()) < VisionConstants.APRILTAG_DISTANCE_THRESHOLD && Math.abs(targetPose.getX() - robotPose.getX()) < VisionConstants.APRILTAG_DISTANCE_THRESHOLD) {
          isAlignmentSuccess = true;
          isAlignmentActive = false; // Stop the alignment when the target is reached
        }
        else {
          isAlignmentSuccess = false;
        }

        //SmartDashboard.putNumber("Commanded X", xCommand);
        //SmartDashboard.putNumber("Commanded Y", yCommand);
        //SmartDashboard.putNumber("Commanded Rot", rotCommand);

        drive(xCommand, yCommand, rotCommand * 0.3, true, true);
      }
  }

  /**
   * A function for driving to the targeted note, runs periodically 
   * 
   * (ONLY USED DURING AUTO)
   */
  public void driveToNote() {
    // Note alignment code
      if (isAlignmentActive && !cameraMode) {
        drive(-VisionConstants.VISION_SPEED_LIMIT, 0, m_noteCamera.getRotationSpeed(), false, true);

        if (!m_noteCamera.getResult().hasTargets()) {
          isAlignmentSuccess = true;
          isAlignmentActive = false; // Stop the alignment when the target is reached
        }
        else {
          isAlignmentSuccess = false;
        }
      }
  }

  // TODO: replace this function with one that makes more sense and go back to calling drive in robotcontainer
  public void driveCommand(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean rateLimit) {

    if (!isAlignmentActive) {
      drive(xSpeed, ySpeed, rotSpeed, fieldRelative, rateLimit);
    }
    
    if (isAlignmentActive && cameraMode == false) {
      // Align to the note while driving normally
      drive(xSpeed, ySpeed, rotSpeed + m_noteCamera.getRotationSpeed(), fieldRelative, rateLimit);
    }

    if (isAlignmentActive && cameraMode == true) {
      // Apriltag alignment code
      if (targetPose != null) {
        Pose2d robotPose = getPose();

        double xCommand = targetPose.getX() - robotPose.getX();
        double yCommand = targetPose.getY() - robotPose.getY();

        double tagRotation = targetPose.getRotation().getDegrees();

        if (tagRotation > 0) {
          tagRotation -= 180;
        }
        else {
          tagRotation += 180;
        }

        double rotCommand = tagRotation - getHeading();
        rotCommand *= Math.abs(rotCommand); // square the rot command for smoother rotation

        // Do not let the commanded speed above a certain value
        if (xCommand > 0) {
          xCommand = Math.min(xCommand, VisionConstants.VISION_SPEED_LIMIT);
        }
        else {
          xCommand = Math.max(xCommand, -VisionConstants.VISION_SPEED_LIMIT);
        }

        if (yCommand < 0) {
          yCommand = Math.max(yCommand, -VisionConstants.VISION_SPEED_LIMIT);
        }
        else {
          yCommand = Math.min(yCommand, VisionConstants.VISION_SPEED_LIMIT);
        }

        if (rotCommand < 0) {
          rotCommand = Math.max(rotCommand, -VisionConstants.VISION_TURN_LIMIT);
        } else {
          rotCommand = Math.min(rotCommand, VisionConstants.VISION_TURN_LIMIT);
        }

        if (Math.abs(rotCommand) < VisionConstants.APRILTAG_ROTATION_THRESHOLD && Math.abs(targetPose.getY() - robotPose.getY()) < VisionConstants.APRILTAG_DISTANCE_THRESHOLD && Math.abs(targetPose.getX() - robotPose.getX()) < VisionConstants.APRILTAG_DISTANCE_THRESHOLD) {
          isAlignmentSuccess = true;
          isAlignmentActive = false; // Stop the alignment when the target is reached
        }
        else {
          isAlignmentSuccess = false;
        }

        SmartDashboard.putNumber("Commanded X", xCommand);
        SmartDashboard.putNumber("Commanded Y", yCommand);
        SmartDashboard.putNumber("Commanded Rot", rotCommand);

        //drive(xCommand, yCommand, rotCommand * 0.3, true, true);
      }
    }
  }

  // If the robot is trying to align with a note or tag this will cancel that
  public void cancelAlign() {
    isAlignmentActive = false;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /*
   * Starts aligning towards a note, if a note can be seen
   */
  public void alignWithNote() {
    SmartDashboard.putBoolean("TEST", true);

    isAlignmentSuccess = false; // Set this to false so the alignment doesn't finish instantly

    cameraMode = false; // Set the camera mode to target notes
    isAlignmentActive = true; // Start aligning
  }

  /*
   * Starts aligning towards a tag, if a tag can be seen
   */
  public void alignWithTag() {
    isAlignmentSuccess = false; // Set this to false so the alignment doesn't finish instantly

    cameraMode = true; // Set the camera mode to target apriltags
    isAlignmentActive = true; // Start aligning
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param maxSpeed      A 0-1 multiplier for the x and y speed of the robot.
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;
    
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Creates an interpolated value based on the min and max speed constants and the position of the slider (m_maxSpeed)
    double lerpSpeed = DriveConstants.kMinSpeedMetersPerSecond + (DriveConstants.kMaxSpeedMetersPerSecond
                     - DriveConstants.kMinSpeedMetersPerSecond) * m_maxSpeed;

    // Convert the commanded speeds into the correct units for the drivetrain,
    // using the interpolated speed
    double xSpeedDelivered = xSpeedCommanded * lerpSpeed;
    double ySpeedDelivered = ySpeedCommanded * lerpSpeed;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
 
    this.setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, m_slowMode ? DriveConstants.kMinSpeedMetersPerSecond : DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Gets the swerve ModuleStates.
   *
   * @return The current SwerveModule states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Sets the idle states of the SparkMAX motors.
   *
   * @param mode the mode to set the states to (0 is coast, 1 is brake)
   */
  public void setIdleStates(int mode) {
    m_frontLeft.setIdle(mode);
    m_rearLeft.setIdle(mode);
    m_frontRight.setIdle(mode);
    m_rearRight.setIdle(mode);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -Infinity to Infinity
   */
  public double getHeading() {
    /* I'm multiplying the navx heading by -1 
    * because WPILib uses CCW as the positive direction
    * and NavX uses CW as the positive direction
    */ 
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Enables and disables slow mode.
   *
   * @param mode Whether to enable slow mode on or off.
   */
  public void setSlowMode(boolean mode) {
    this.m_slowMode = mode;
  }

  /**
   * Sets the speed of the robot to a percentage [0 --> 1]
   *
   * @param percent The desired speed percentage
   */
  public void setSpeedPercent(double percent) {
      m_maxSpeed = percent;
  }

  /**
   * Initializes the auto using PathPlannerLib.
   */
  public void initializeAuto() {
    AutoBuilder.configureHolonomic(
          this::getPose, 
          this::resetOdometry, 
          this::getChassisSpeeds, 
          this::setChassisSpeeds,
          new HolonomicPathFollowerConfig( 
                  new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                  new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                  AutoConstants.kMaxSpeedMetersPerSecond,
                  AutoConstants.kDriveBase, // Distance from robot center to furthest module
                  new ReplanningConfig() 
          ),
          () -> {
              Optional<Alliance> alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
          },
          this   // Reference to this subsystem to set requirements
        );
  }
  
  /**
   * Sets the speed of the robot chassis.
   * @param speed The new chassis speed.
   */
  public void setChassisSpeeds(ChassisSpeeds speed) {
    this.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speed));
  }

  public void alignAuto() {
    if (cameraMode && isAlignmentActive) {
      driveToTag();
    }
    else if (isAlignmentActive) {
      driveToNote();
    }
  }

  /**
   * Gets the speed of the robot chassis.
   * @return The current chassis speed.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(this.getModuleStates());
  }

  /** Prints all values to dashboard */
  public void printToDashboard() {
    // Speed
    SmartDashboard.putNumber("Vertical Speed", this.getChassisSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Horizontal Speed", this.getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Turn Speed", this.getChassisSpeeds().omegaRadiansPerSecond);
    SmartDashboard.putNumber("Current Speed Percentage", m_maxSpeed);

    // Position
    SmartDashboard.putNumber("X Position", this.getPose().getX());
    SmartDashboard.putNumber("Y Position", this.getPose().getY());
    SmartDashboard.putNumber("Gyro Heading: ", this.getHeading());
    SmartDashboard.putNumber("Odometry Heading: ", this.m_odometry.getPoseMeters().getRotation().getDegrees());

    // Slew rate filter variables
    SmartDashboard.putNumber("slewCurrentRotation: ", m_currentRotation);
    SmartDashboard.putNumber("slewCurrentTranslationDirection: ", m_currentTranslationDir);
    SmartDashboard.putNumber("slewCurrentTranslationMagnitude: ", m_currentTranslationMag);

    // Encoder values
    SmartDashboard.putString("Front left Encoder", m_frontLeft.getState().toString());
    SmartDashboard.putString("Front right Encoder", m_frontRight.getState().toString());
    SmartDashboard.putString("Rear left Encoder", m_rearLeft.getState().toString());
    SmartDashboard.putString("Rear right Encoder", m_rearRight.getState().toString());

    SmartDashboard.putBoolean("Align", isAlignmentActive);
    SmartDashboard.putBoolean("Alignment Success", isAlignmentSuccess);
    SmartDashboard.putBoolean("Camera Mode", cameraMode);

    // Apriltag target location/rotation (field relative space)
    if (targetPose != null) {
      SmartDashboard.putNumber("Target X", targetPose.getX());
      SmartDashboard.putNumber("Target Y", targetPose.getY());

      SmartDashboard.putNumber("Target Rotation", targetPose.getRotation().getDegrees());
    }

    // Do the cameras have targets?
    SmartDashboard.putBoolean("Note Cam", m_noteCamera.getResult().hasTargets());
    SmartDashboard.putBoolean("Tag Cam", m_tagCamera.getResult().hasTargets());
  }
}
