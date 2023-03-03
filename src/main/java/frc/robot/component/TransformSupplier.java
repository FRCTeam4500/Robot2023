package frc.robot.component;


import frc.robot.utility.Transform3D;

/**
 * Component which returns the transform of the robot
 *
 */
public interface TransformSupplier {
    /**
     *
     * @return The transform of the robot (i think this means location but i'm honestly not sure)
     */
    public Transform3D getTransform();
}
