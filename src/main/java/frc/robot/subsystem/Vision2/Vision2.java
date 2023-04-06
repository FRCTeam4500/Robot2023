package frc.robot.subsystem.Vision2;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.component.hardware.LimelightVisionComponent;

public class Vision2 extends SubsystemBase{
    private LimelightVisionComponent camera1;

    public Vision2(LimelightVisionComponent vision1) {
        camera1 = vision1;
    }

    public boolean hasValidTargets() {
        return camera1.hasValidTargets();
    }

    public double depthToTarget() {
        return (VisionConstants.TARGET_HEIGHT - VisionConstants.VISION_HEIGHT) / Math.tan(Math.abs(camera1.getVerticalOffsetFromCrosshair()));
    }


}
