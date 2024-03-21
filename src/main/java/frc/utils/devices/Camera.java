package frc.utils.devices;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;
import frc.utils.VisionUtils;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose2d;

public class Camera extends PhotonCamera {

    private double rotationSpeed;
    private PIDController alignController;

    /**
     * Constructor for the Camera class
     * @param name the name of the camera
     */
    public Camera(String name) {
        super(name);
        alignController = new PIDController(VisionConstants.kNoteP, VisionConstants.kNoteI, VisionConstants.kNoteD);
    }

    PhotonPipelineResult result = this.getLatestResult();
    
    /**
     * Gets the desired rotation speed in order to align with the target
     * @return the desired rotation speed
     */
    public double getRotationSpeed(){
        // Make sure the camera has a target, else it will return null
        if(result.hasTargets()){
            double currentYaw = result.getBestTarget().getYaw();

            rotationSpeed = alignController.calculate(currentYaw);
            
        } else{
            rotationSpeed = 0;
        }

        return rotationSpeed;
    }

    /**
     * Gets the result of the camera
     * @return the result of the camera
     */
    public PhotonPipelineResult getResult() {
        return result;
    }

    /**
     * Refreshes the result of the camera
     */
    public void refreshResult() {
        result = this.getLatestResult();
    }

    /**
     * Finds the offset of a target with a specific id
     * @return The offset of the specified target
     */
    public Transform3d findTagWithId(int desiredId) {
        List<PhotonTrackedTarget> targetData = result.getTargets();
        
        for (PhotonTrackedTarget currentTarget : targetData) {
            if (currentTarget.getFiducialId() == desiredId) {
                return currentTarget.getBestCameraToTarget();
            }
        }

        return null;
    }

    /**
     * Determines whether the camera sees an apriltag of a specific id
     * @return Whether there is a target with the desired id
     */
    public boolean hasTargetOfId(int desiredId) {
        List<PhotonTrackedTarget> targetData = result.getTargets();
        
        for (PhotonTrackedTarget currentTarget : targetData) {
            if (currentTarget.getFiducialId() == desiredId) {
                return true;
            }
        }

        return false;
    }

    /**
     * Gets the transform of the target
     * @param robotHeading the heading of the robot
     * @return the transform of the target
     */
    public Transform2d getTargetTransform(double robotHeading, int tagToTarget) {
        Transform3d rawOffset = findTagWithId(tagToTarget);

        // Only return the pose if there is actually a target
        if (result.hasTargets()) {
            Transform2d robotRelativeOffset = new Transform2d(rawOffset.getX(), 
            rawOffset.getY(), 
            new Rotation2d(rawOffset.getRotation().getZ()));

            return VisionUtils.toFieldTransform(robotRelativeOffset, robotHeading);
        }
        else {
            return null;
        }
    }

    /**
     * A function for getting the field position of a tracked apriltag
     * @param robotPose the pose of the robot
     * @param robotHeading the heading of the robot
     * @return the pose of the apriltag
     */
    public Pose2d getApriltagPose(Pose2d robotPose, double robotHeading, int tagId, double tagHeading) {
        Transform2d robotToTag = getTargetTransform(robotHeading, tagId);

        // Add the robot to tag offset to the robot pose to get the tag pose in field space
        Pose2d tagPose = new Pose2d(robotPose.getX() + 
        robotToTag.getX(), robotPose.getY() + 
        robotToTag.getY(), Rotation2d.fromDegrees(tagHeading));

        return tagPose;
    }

    /**
     * Sets the pipeline index of the camera
     * @param pipelineIndex the index of the pipeline
     */
    public void setCameraPipeline(int pipelineIndex) {
        this.setPipelineIndex(pipelineIndex);
    }

    /**
     * Gets the pipeline index of the camera
     * @return the index of the pipeline
     */
    public int getCameraPipeline() {
        return this.getPipelineIndex();
    }
}
