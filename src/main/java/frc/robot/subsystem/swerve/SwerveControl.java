package frc.robot.subsystem.swerve;

/**
 * Control class for Swerve, allows for communication between a RobotContainer and Swerve
 * A RobotContainer should set these values in order to tall the swerve which direction to move
 * The swerve default command will then look at these values and move the swerve
 */

public class SwerveControl {
    public double x = 0;
    public double y = 0;
    public double z = 0;
}
