package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.component.hardware.LimelightVisionComponent;

public class Vision2 extends SubsystemBase{
    private LimelightVisionComponent camera1;
    
    public Vision2(LimelightVisionComponent vision1) {
        camera1 = vision1;
    }


}
