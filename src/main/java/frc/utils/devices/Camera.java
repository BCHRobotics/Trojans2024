package frc.utils.devices;

import static frc.robot.Constants.VisionConstants.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose2d;

public class Camera extends PhotonCamera {
    private final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;
    private double rotationSpeed;
    private PIDController alignController;

    /**
     * Constructor for the Camera class
     * @param name the name of the camera
     */
    public Camera(String name) {
        super(name);
        alignController = new PIDController(VisionConstants.kNoteP, VisionConstants.kNoteI, VisionConstants.kNoteD);
        photonEstimator =
                new PhotonPoseEstimator(
                        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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

            return toFieldTransform(robotRelativeOffset, robotHeading);
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
    public Pose2d getApriltagPose(Pose2d robotPose, double robotHeading, int tagId) {
        Transform2d robotToTag = getTargetTransform(robotHeading, tagId);

        // Add the robot to tag offset to the robot pose to get the tag pose in field space
        Pose2d tagPose = new Pose2d(robotPose.getX() + 
        robotToTag.getX(), robotPose.getY() + 
        robotToTag.getY(), new Rotation2d((robotHeading + robotToTag.getRotation().getDegrees()) * (Math.PI / 180)));

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

    /**
     * A function that converts the supplied Transform2d in object relative coordinates 
     * (object as in a tag or robot or something else that points in a direction)
     * into a Transform2d in field relative coordinates.
     * @param objectTransform the transform of the object
     * @param heading the heading of the robot
     * @return the transform of the object in field coordinates
     */
    public Transform2d toFieldTransform(Transform2d objectTransform, double heading) {
        // Multiply the heading by PI/180 to convert to radians
        double sinHeading = Math.sin(heading * (Math.PI / 180));
        double cosHeading = Math.cos(heading * (Math.PI / 180));

        double fieldX = objectTransform.getX() * cosHeading + objectTransform.getY() * -sinHeading;
        double fieldY = objectTransform.getX() * sinHeading + objectTransform.getY() * cosHeading;

        Transform2d fieldTransform = new Transform2d(-fieldX, -fieldY, objectTransform.getRotation());

        return fieldTransform;
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
        double latestTimestamp = this.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}
