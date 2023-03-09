/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.component.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.component.SmartMotorComponent;

/**
 * An {@link OutputSetterComponent}, {@link AngleSetterComponent}, and
 * {@link AngleGetterComponent} wrapper for {@link TalonSRX}.
 */
public class TalonSRXComponent extends TalonSRX
        implements SmartMotorComponent {

    public static final double TICKS_PER_DEGREE = 4096 / 360.0;
    public static final double TICKS_PER_RADIAN = 4096 / Math.PI / 2.0;
    public static final double TICKS_PER_REVOLUTION = 4096;

    /**
     * @see TalonSRX#TalonSRX(int)
     */
    public TalonSRXComponent(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void setOutput(double speed) {
        set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setAngle(double angle) {
        set(ControlMode.Position, angle);
    }

    @Override
    public double getAngle() {
        return getSelectedSensorPosition();
    }

    @Override
    public double getAngularVelocity() {
        return getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10;
    }

    @Override
    public double getOutput() {
        return getMotorOutputPercent();
    }

    @Override
    public void setAngularVelocity(double velocity) {
        set(ControlMode.Velocity, velocity * TICKS_PER_RADIAN / 10.0);
    }
}
