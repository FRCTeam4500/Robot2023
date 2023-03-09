package frc.robot.subsystem.vision;

import frc.robot.component.hardware.LimelightVisionComponent;

public class HardwareVisionFactory {
    public static Vision makeVision(){
        double height = 0.6096;
        double angle = Math.toRadians(38);
        Vision vision = new VisionImpl(new LimelightVisionComponent(), height, angle);
        return vision;
    }
}
