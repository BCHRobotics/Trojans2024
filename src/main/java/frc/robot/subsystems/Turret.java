package frc.robot.subsystems;
import java.lang.Math;

public class Turret {
    public double calculateTurretAngle (double x,double y, double velocity) {
        double g = 9.81;  // Acceleration due to gravity
        
        double discriminant = Math.pow(velocity, 4) - g * (g * Math.pow(x,2) + 2 * y * Math.pow(velocity, 2));
        if (discriminant < 0){
            return 0;  // No real solution
        }
        else{
            double theta1 = Math.atan((Math.pow(velocity, 2) + Math.sqrt(discriminant)) / (g * x));
            double theta2 = Math.atan((Math.pow(velocity, 2)- Math.sqrt(discriminant)) / (g * x));
            return Math.min(theta1, theta2);
        }
    }
}
