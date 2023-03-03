package frc.robot.component;

/**
 * A component which controls angular velocity
 */
public interface AngularVelocityComponent {
    /**
     * Changes the angular velocity
     * @param angularVelocity The new angular velocity
     */
    public void setAngularVelocity(double angularVelocity);

    /**
     *
     * @return The angular velocity
     */
    public double getAngularVelocity();
}
