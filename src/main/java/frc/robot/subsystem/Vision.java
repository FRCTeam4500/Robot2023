package frc.robot.subsystem;

import frc.robot.component.VisionComponent;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.component.hardware.LimelightVisionComponent;
import frc.robot.Constants.VisionConstants;

public class Vision implements Subsystem {
    private  VisionComponent visionComponent;
    private double height;
    private double angle;
    private double previousOffset = 0;

    static final double maximumAllowableOffset = Units.degreesToRadians(5);

    /* Vision from specified "constants" */
    public Vision(VisionComponent camera, double height, double angle) {
        visionComponent = camera;
        this.height = height;
        this.angle = angle;
    }

    public boolean hasValidTargets() {
        return visionComponent.hasValidTargets();
    }

    public double getHorizontalOffsetFromCrosshair() {
        double currentOff = visionComponent.getHorizontalOffsetFromCrosshair();
        double avgOff = (currentOff + previousOffset) / 2;
        previousOffset = currentOff;
        return avgOff;
    }

    public double getVerticalOffsetFromCrosshair() {
        return visionComponent.getVerticalOffsetFromCrosshair();
    }

    public double getTargetArea() {
        return visionComponent.getTargetArea();
    }

    public double getSkew() {
        return visionComponent.getSkew();
    }

    public double getVisionAngle() {
        return angle;
    }

    public double getVisionHeight() {
        return height;
    }

    public static Vision makeVision() {
        return new Vision(new LimelightVisionComponent(), VisionConstants.height, VisionConstants.angle);
    }

    public class VisionDistanceCalculator {
        static double targetHeight = VisionConstants.TargetHeight;
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
    
}
