package frc.utils.devices;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;

public class Camera {
    double rotationSpeed;
    private static PhotonCamera instance;

    public static PhotonCamera getInstance() {
        if (instance == null) { 
            instance = new PhotonCamera("Photon"); // replace with the name of the camera which is set in the UI
        }
        return instance;
    }

    PIDController turnController = new PIDController(0.25
    , 0.3, 0);

    PhotonPipelineResult result = getInstance().getLatestResult();

    double prevYaw;
    
    public double getRotationSpeed(){
        if(result.hasTargets()){
            double currentYaw = result.getBestTarget().getYaw() * 0.05;
            double lerpValue = (prevYaw + currentYaw) /  2;

            rotationSpeed = turnController.calculate(result.getBestTarget().getYaw() * 0.05, 0);

            prevYaw = currentYaw;
        } else{
            rotationSpeed = 0;
        }
        return rotationSpeed;
    }

    public PhotonPipelineResult getResult() {
        return result;
    }

    public void refreshResult() {
        result = getInstance().getLatestResult();
    }
}
