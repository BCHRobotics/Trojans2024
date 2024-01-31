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
            instance = new PhotonCamera("photonvision"); // replace with the name of the camera which is set in the UI
        }
        return instance;
    }

    PIDController turnController = new PIDController(0.3, 0, 0);

    PhotonPipelineResult result = instance.getLatestResult();
    
    public double getRotationSpeed(){
        if(result.hasTargets()){
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else{
            rotationSpeed = 0;
        }
        return rotationSpeed;
    }


}
