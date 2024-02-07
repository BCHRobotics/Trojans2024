package frc.utils.devices;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose2d;

public class Camera {
    double rotationSpeed;
    private static PhotonCamera instance;

    public static PhotonCamera getInstance() {
        if (instance == null) { 
            instance = new PhotonCamera("Photon"); // replace with the name of the camera which is set in the UI
        }
        return instance;
    }

    PIDController turnController = new PIDController(0.05
    , 0.25, 0);

    PhotonPipelineResult result = getInstance().getLatestResult();
    
    // Gets the desired rotation speed in order to align with the target
    public double getRotationSpeed(){
        // Make sure the camera has a target, else it will return null
        if(result.hasTargets()){
            double currentYaw = result.getBestTarget().getYaw();

            rotationSpeed = turnController.calculate(currentYaw * 0.075, 0);
        } else{
            rotationSpeed = 0;
        }

        return rotationSpeed;
    }

    // Gets the result of the camera
    public PhotonPipelineResult getResult() {
        return result;
    }

    // Refresh the camera's result
    public void refreshResult() {
        result = getInstance().getLatestResult();
    }

    // Returns the pose of the target
    Transform3d getTargetTransform3d() {
        // Only return the pose if there is actually a target
        if (result.hasTargets()) {
            return result.getBestTarget().getBestCameraToTarget();
        }
        else {
            return null;
        }
    }

    public Transform2d getTargetTransform2d(double robotHeading) {
        // Only return the pose if there is actually a target
        if (result.hasTargets()) {
            Transform2d robotRelativeOffset = new Transform2d(getTargetTransform3d().getX(), 
            getTargetTransform3d().getY(), 
            new Rotation2d(getTargetTransform3d().getRotation().getZ()));

            return robotToFieldTransform(robotRelativeOffset, robotHeading);
        }
        else {
            return null;
        }
    }

    // A function for getting the field position of a tracked apriltag
    public Pose2d getApriltagPose(Pose2d robotPose, double robotHeading) {
        // Make sure the camera is currently tracking an apriltag before getting pose data
        if (instance.getPipelineIndex() == VisionConstants.APRILTAG_PIPELINE) {
            Pose2d tagPose = robotPose.plus(getTargetTransform2d(robotHeading));

            return tagPose;
        }
        else {
            return null;
        }
    }

    // A function for setting the pipeline of the camera
    public void setCameraPipeline(int pipelineIndex) {
        instance.setPipelineIndex(pipelineIndex);
    }

    // A function for getting the pipeline of the camera
    public int getCameraPipeline() {
        return instance.getPipelineIndex();
    }

    /*
     * A function that converts the supplied Transform2d in robot relative coordinates 
     * into a Transform2d in field relative coordinates using sin and cos.
     */

    /* 
    public Transform2d robotToFieldTransform(Transform2d robotTransform, double robotHeading) {
        Transform2d fieldTransform = 
        new Transform2d(robotTransform.getX() * Math.cos(robotHeading * (Math.PI / 180))
         + robotTransform.getY() * Math.sin(robotHeading * (Math.PI / 180)), 
         robotTransform.getX() * Math.sin(robotHeading * (Math.PI / 180))
         + robotTransform.getY() * Math.cos(robotHeading * (Math.PI / 180)),
         robotTransform.getRotation());

        return fieldTransform;
    }
    */
    
    public Transform2d robotToFieldTransform(Transform2d robotTransform, double robotHeading){
        double cosHeading = Math.cos(robotHeading * (Math.PI / 180));
        double sinHeading = Math.sin(robotHeading * (Math.PI / 180));

        double fieldX = robotTransform.getX() * cosHeading + robotTransform.getY() * sinHeading;
        double fieldY = robotTransform.getY() * sinHeading + robotTransform.getX() * cosHeading;

        Transform2d fieldTransform = new Transform2d(fieldX, fieldY, robotTransform.getRotation());

        return fieldTransform;
    }
}
