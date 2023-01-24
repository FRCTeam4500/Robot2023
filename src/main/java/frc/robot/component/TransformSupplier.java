package frc.robot.component;


import frc.robot.utility.Transform3D;

/**
 * Component which returns the transform of the robot
 *
 *
 */
public interface TransformSupplier {
    public Transform3D getTransform();
}
