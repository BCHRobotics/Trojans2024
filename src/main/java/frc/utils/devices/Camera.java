package frc.utils.devices;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose2d;

public class Camera extends PhotonCamera {

    private double rotationSpeed;
    private PIDController alignController;

    public Camera(String name) {
        super(name);
        alignController = new PIDController(VisionConstants.kNoteP, VisionConstants.kNoteI, VisionConstants.kNoteD);
    }

    PhotonPipelineResult result = this.getLatestResult();
    
    // Gets the desired rotation speed in order to align with the target
    public double getRotationSpeed(){
        // Make sure the camera has a target, else it will return null
        if(result.hasTargets()){
            double currentYaw = result.getBestTarget().getYaw();

            rotationSpeed = alignController.calculate(currentYaw - 10);
            
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
        result = this.getLatestResult();
    }

    public Transform2d getTargetTransform(double robotHeading) {
        Transform3d rawOffset = result.getBestTarget().getBestCameraToTarget();

        // Only return the pose if there is actually a target
        if (result.hasTargets()) {
            Transform2d robotRelativeOffset = new Transform2d(rawOffset.getX(), 
            rawOffset.getY(), 
            new Rotation2d(rawOffset.getRotation().getZ()));

            return toFieldTransform(robotRelativeOffset, robotHeading);
        }
        else {
            return null;
        }
    }

    // A function for getting the field position of a tracked apriltag
    public Pose2d getApriltagPose(Pose2d robotPose, double robotHeading) {
        Transform2d robotToTag = getTargetTransform(robotHeading);

        // Addd the robot to tag offset to the robot pose to get the tag pose in field space
        Pose2d tagPose = new Pose2d(robotPose.getX() + 
        robotToTag.getX(), robotPose.getY() + 
        robotToTag.getY(), new Rotation2d((robotHeading + robotToTag.getRotation().getDegrees()) * (Math.PI / 180)));

        return tagPose;
    }

    // A function for setting the pipeline index of the camera
    public void setCameraPipeline(int pipelineIndex) {
        this.setPipelineIndex(pipelineIndex);
    }

    // A function for getting the pipeline index of the camera
    public int getCameraPipeline() {
        return this.getPipelineIndex();
    }

    /*
     * A function that converts the supplied Transform2d in object relative coordinates 
     * (object as in a tag or robot or something else that points in a direction)
     * into a Transform2d in field relative coordinates.
     */
    public Transform2d toFieldTransform(Transform2d objectTransform, double heading) {
        // Multiply the heading by PI/180 to convert to radians
        double sinHeading = Math.sin(heading * (Math.PI / 180));
        double cosHeading = Math.cos(heading * (Math.PI / 180));

        // Here object transform is negative, because the apriltag camera is on the back of the robot
        // For a front camera make it positive
        double fieldX = -objectTransform.getX() * cosHeading + -objectTransform.getY() * -sinHeading;
        double fieldY = -objectTransform.getX() * sinHeading + -objectTransform.getY() * cosHeading;

        Transform2d fieldTransform = new Transform2d(fieldX, fieldY, objectTransform.getRotation());

        return fieldTransform;
    }
}
