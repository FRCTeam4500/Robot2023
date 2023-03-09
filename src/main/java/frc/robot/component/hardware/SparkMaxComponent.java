
package frc.robot.component.hardware;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.component.SmartMotorComponent;

/**
 * Wrapper for CANSparkMax motor controller which implements SmartMotorComponent
 */
public class SparkMaxComponent extends CANSparkMax implements SmartMotorComponent {

    public SparkMaxComponent(int deviceID, MotorType type) {
        super(deviceID, type);
    }

    
    public double getAngle() {
        return getEncoder().getPosition();
    }

    
    public void setAngle(double position) {
        getPIDController().setReference(position, ControlType.kPosition);

    }

    @Override
    public void setOutput(double output) {
        set(output);
    }

    @Override
    public double getOutput() {
        return get();
    }

    @Override
    public double getAngularVelocity() {
        return Units.rotationsPerMinuteToRadiansPerSecond(getEncoder().getVelocity());
    }

    /**
     * set velocity, in rad/s
     * @param velocity angular velocity, in rad/s
     */
    @Override
    public void setAngularVelocity(double velocity) {
        getPIDController().setReference(Units.radiansPerSecondToRotationsPerMinute(velocity), ControlType.kVelocity);
    }

}
