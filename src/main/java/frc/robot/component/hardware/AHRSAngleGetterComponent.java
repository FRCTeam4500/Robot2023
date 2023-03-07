/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.component.hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.component.GyroComponent;

/**
 * An {@link GyroComponent} wrapper for {@link AHRS}.
 * All getters return radians.
 */
public class AHRSAngleGetterComponent extends AHRS implements GyroComponent {

    /**
     * @see AHRS#AHRS(Port)
     */
    public AHRSAngleGetterComponent(edu.wpi.first.wpilibj.I2C.Port kmxp) {
        super(kmxp);
    }
    public AHRSAngleGetterComponent(Port kmxp) {
        super(kmxp);
    }

    @Override
    public double getAngle() {
        return -Math.toRadians(super.getAngle());
    }

    @Override
    public float getPitch() {
        return (float) Math.toRadians(super.getPitch());
    }

    @Override
    public float getRoll() {
        return (float) Math.toRadians(super.getRoll());
    }

    public void reset() {}
}
