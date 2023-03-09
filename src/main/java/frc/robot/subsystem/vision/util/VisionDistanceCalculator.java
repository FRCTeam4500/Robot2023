package frc.robot.subsystem.vision.util;

import frc.robot.subsystem.vision.Vision;

public class VisionDistanceCalculator {
    static double targetHeight = 2.64;
    static double lastValidOffset = 0;

    /**
     * Calculates distance from target using trigonometry
     * @param vision The vision
     * @return the horizontal distance from the target, in meters
     */
    public static double calculateDistance(Vision vision){

        double offset = vision.getVerticalOffsetFromCrosshair(); //angle offset, in radians
        double heightDiff = targetHeight - vision.getVisionHeight(); //opposite side of triangle
        double distance = heightDiff / Math.tan(vision.getVisionAngle() + offset);
        return distance;
    }
}
