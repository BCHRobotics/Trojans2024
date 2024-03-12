package frc.utils;

public class CustomPoseEstimator {
    private static final double DEG_TO_RAD = Math.PI / 180.0;

    public static double[] findCameraPosition(double cameraFacingAngle, double fieldWidth, double fieldHeight,
                                              double pointX, double pointY, double cameraToPointAngle) {
        // Convert angles to radians
        double cameraFacingAngleRad = cameraFacingAngle * DEG_TO_RAD;
        double cameraToPointAngleRad = cameraToPointAngle * DEG_TO_RAD;

        // Calculate the third angle of the triangle
        double thirdAngleRad = Math.PI - cameraToPointAngleRad - cameraFacingAngleRad;

        // Use the law of sines to determine the ratio between sides
        double ratio = Math.sin(thirdAngleRad) / Math.sin(cameraToPointAngleRad);

        // Calculate intersection point on the top wall
        double intersectionX = pointY * ratio;

        // Calculate the camera's distance from the point on the wall (law of sines)
        double cameraDistanceToPoint = pointY / Math.tan(cameraToPointAngleRad);

        // Calculate the camera's x and y position
        double cameraX = cameraDistanceToPoint * Math.cos(cameraToPointAngleRad);
        double cameraY = cameraDistanceToPoint * Math.sin(cameraToPointAngleRad) + pointY;

        // Return the calculated camera coordinates
        return new double[] {cameraX, cameraY};
    }
}
