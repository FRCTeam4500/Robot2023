/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.component;

/**
 * An interface for any component that gets the angle of something, for example,
 * an encoder. For gyros, use {@link GyroComponent}, as it contains methods
 * such as {@link GyroComponent#reset()} for better gyro control.
 */
public interface AngleComponent extends AngleGetterComponent{
    /**
     * Gets the measured angle in radians. This angle is measured using standard
     * angles (east is zero, positive is counter clockwise, negative is clockwise)
     * unless otherwise specified or if standard angles do not make sense for this
     * use case.
     *
     * @return radians of measured angle
     */
    double getAngle();

    /**
     * Sets the measured angle in radians. This angle is measured using standard
     * angles (east is zero, positive is counter clockwise, negative is clockwise)
     * unless otherwise specified or if standard angles do not make sense for this
     * use case.
     *
     * @return radians of measured angle
     * @param v
     */
    void setAngle(double v);
}