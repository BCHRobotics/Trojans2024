package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraModes;

public class VisionUtils {

    /**
     * A function that converts the supplied Transform2d in object relative coordinates 
     * (object as in a tag or robot or something else that points in a direction)
     * into a Transform2d in field relative coordinates.
     * @param objectTransform the transform of the object
     * @param heading the heading of the robot
     * @return the transform of the object in field coordinates
     */
    // TODO: test if inputting a negative heading does the same thing as flipping the result
    public static Transform2d toFieldTransform(Transform2d objectTransform, double heading) {
        // Multiply the heading by PI/180 to convert to radians
        double sinHeading = Math.sin(heading * (Math.PI / 180));
        double cosHeading = Math.cos(heading * (Math.PI / 180));

        double fieldX = objectTransform.getX() * cosHeading + objectTransform.getY() * -sinHeading;
        double fieldY = objectTransform.getX() * sinHeading + objectTransform.getY() * cosHeading;

        Transform2d fieldTransform = new Transform2d(-fieldX, -fieldY, objectTransform.getRotation());

        return fieldTransform;
    }

    /**
     * A function for calculating a the movement towards a given apriltag's 
     * exact position (USED FOR THE AMP)
     * @param targetPose the pose of the apriltag
     * @param robotPose the pose of the robot
     * @param desiredOffset the desired offset from the apriltag
     * @return
     */
    public static Transform2d alignWithTagExact(Pose2d targetPose, Pose2d robotPose, Transform2d desiredOffset) {
      // Apriltag alignment code
      if (targetPose != null) {
        // Calculate the x and y commands based on the direction to the tag
        double xCommand = targetPose.getX() + desiredOffset.getX() - robotPose.getX();
        double yCommand = targetPose.getY() + desiredOffset.getY() - robotPose.getY();

        // Get the heading of the tag based on the camera mode (red/blue)
        Rotation2d tagRotation = Rotation2d.fromDegrees(-90);
        // Robot heading
        Rotation2d robotRotation = robotPose.getRotation();
        // Commanded rotation
        double rotCommand = tagRotation.minus(robotRotation).getDegrees();

        // x axis command
        xCommand = (xCommand < 0) ? 
        Math.max(xCommand, -VisionConstants.kVisionSpeedLimit) : 
        Math.min(xCommand, VisionConstants.kVisionSpeedLimit);

        // y axis command
        yCommand = (yCommand < 0) ? 
        Math.max(yCommand, -VisionConstants.kVisionSpeedLimit) :
        Math.min(yCommand, VisionConstants.kVisionSpeedLimit);
        
        // Rotational command
        rotCommand = (rotCommand < 0) ? 
        Math.max(rotCommand, -VisionConstants.kVisionTurningLimit) :
        Math.min(rotCommand, VisionConstants.kVisionTurningLimit);

        // Set the x command to full speed if far enough
        if (Math.abs(xCommand) > VisionConstants.kTagSlowdownDistance) {
            xCommand = (xCommand < 0) ? 
            -VisionConstants.kVisionSpeedLimit : 
            VisionConstants.kVisionSpeedLimit;
        }

        // Set the y command to full speed if far enough
        if (Math.abs(yCommand) > VisionConstants.kTagSlowdownDistance) {
            yCommand = (yCommand < 0) ? 
            -VisionConstants.kVisionSpeedLimit : 
            VisionConstants.kVisionSpeedLimit;
        }

        // Check if robot is within acceptable boundaries
        boolean rotFinished = Math.abs(tagRotation.getDegrees() - robotPose.getRotation().getDegrees()) < VisionConstants.kTagRotationThreshold;
        boolean xFinished = Math.abs(targetPose.getX() + desiredOffset.getX() - robotPose.getX()) < VisionConstants.kTagDistanceThreshold;
        boolean yFinished = Math.abs(targetPose.getY() + desiredOffset.getY() - robotPose.getY()) < VisionConstants.kTagDistanceThreshold;

        if (rotFinished) { rotCommand = 0; }
        if (xFinished) { xCommand = 0; }
        //if (yFinished) { yCommand = 0; }

        if (rotFinished && xFinished && yFinished) {
          return null;
        }
        else {
          // TODO: Maybe get rid of the * 0.3?
          return new Transform2d(xCommand, yCommand, Rotation2d.fromDegrees(rotCommand * 0.3));
        }
      }
      else {
        return null;
      }
    }

    /**
     * A function for calculating a the movement towards a circle 
     * defined by a given apriltag (USED FOR THE SPEAKER)
     * @param targetPose the pose of the apriltag
     * @param robotPose the pose of the robot
     * @return
     */
    //TODO: finish this function
    public static Transform2d alignWithTagRadial(Pose2d targetPose, Pose2d robotPose, double desiredRadius) {
        // Apriltag alignment code
        if (targetPose != null) {
          double distToTag = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        
          // Calculate the x and y commands, based on whether the robot should travel away or towards the tag
          double xCommand = (distToTag > desiredRadius) ? (targetPose.getX() - robotPose.getX()) : (targetPose.getX() - robotPose.getX()) * -1;
          double yCommand = (distToTag > desiredRadius) ? (targetPose.getY() - robotPose.getY()) : (targetPose.getY() - robotPose.getY()) * -1;
          
          // TODO: test the rotation here
          // Commanded rotation based on direction vector
          double rotCommand = Math.atan((targetPose.getY() - robotPose.getY()) / (targetPose.getX() - robotPose.getX()));
  
          // x axis command
          xCommand = (xCommand < 0) ? 
          Math.max(xCommand, -VisionConstants.kVisionSpeedLimit) : 
          Math.min(xCommand, VisionConstants.kVisionSpeedLimit);
  
          // y axis command
          yCommand = (yCommand < 0) ? 
          Math.max(yCommand, -VisionConstants.kVisionSpeedLimit) :
          Math.min(yCommand, VisionConstants.kVisionSpeedLimit);
          
          // Rotational command
          rotCommand = (rotCommand < 0) ? 
          Math.max(rotCommand, -VisionConstants.kVisionTurningLimit) :
          Math.min(rotCommand, VisionConstants.kVisionTurningLimit);
  
          // Set the x command to full speed if far enough
          if (Math.abs(xCommand) > VisionConstants.kTagSlowdownDistance) {
              xCommand = (xCommand < 0) ? 
              -VisionConstants.kVisionSpeedLimit : 
              VisionConstants.kVisionSpeedLimit;
          }
  
          // Set the y command to full speed if far enough
          if (Math.abs(yCommand) > VisionConstants.kTagSlowdownDistance) {
              yCommand = (yCommand < 0) ? 
              -VisionConstants.kVisionSpeedLimit : 
              VisionConstants.kVisionSpeedLimit;
          }
  
          // Check if robot is within acceptable boundaries
          boolean rotFinished = Math.abs(Math.atan((targetPose.getY() - robotPose.getY()) / (targetPose.getX() - robotPose.getX()))) < VisionConstants.kTagRotationThreshold;
          boolean posFinished = Math.abs(distToTag - desiredRadius) < VisionConstants.kTagDistanceThreshold;
  
          if (rotFinished && posFinished) {
              return new Transform2d(0, 0, Rotation2d.fromDegrees(0));
          }
  
          // TODO: Maybe get rid of the * 0.3?
          return new Transform2d(xCommand, yCommand, Rotation2d.fromDegrees(rotCommand * 0.3));
        }
        else {
          return new Transform2d(0, 0, Rotation2d.fromDegrees(0));
        }
    }
}
