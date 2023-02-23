package frc.robot.subsystem.vision;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface for vision subsystems
 */

public interface Vision extends Subsystem {
    boolean hasValidTargets();

    double getHorizontalOffsetFromCrosshair();

    double getVerticalOffsetFromCrosshair();

    double getTargetArea();

    double getSkew();

    /**
     * The vision itself should hold height and angle because they are hardware dependent
     * The constants class should only hold configuration values
     */
    double getVisionHeight();

    double getVisionAngle();
}
