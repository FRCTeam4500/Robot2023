package frc.robot.component;

public class DoubleMotorComponent implements SmartMotorComponent {

    private SmartMotorComponent m1, m2;

    public DoubleMotorComponent(SmartMotorComponent motor1, SmartMotorComponent motor2) {
        m1 = motor1;
        m2 = motor2;
    }

    public double getAngle() {
        return m1.getAngle();
    }

    public void setAngle(double angle) {
        m1.setAngle(angle);
        m2.setAngle(angle);
    }

    public double getOutput() {
        return m1.getOutput();
    }

    public void setOutput(double output) {
        m1.setOutput(output);
        m2.setOutput(output);
    }

    public double getAngularVelocity() {
        return m1.getAngularVelocity();
    }

    public void setAngularVelocity(double angularVelocity) {
        m1.setAngularVelocity(angularVelocity);
        m2.setAngularVelocity(angularVelocity);
    }

}