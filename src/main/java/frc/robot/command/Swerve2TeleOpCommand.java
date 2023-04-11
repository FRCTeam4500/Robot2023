package frc.robot.command;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.Swerve2.Swerve2;

public class Swerve2TeleOpCommand extends CommandBase{
    private Swerve2 swerve;
    private CommandXboxController controller;

    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter wLimiter;

    private double xSens;
    private double ySens;
    private double wSens;

    private boolean lockRotation;
    private double targetAngle;
    public Swerve2TeleOpCommand(Swerve2 swerve, CommandXboxController controller) {
        this.swerve = swerve;
        this.controller = controller;
        xLimiter = new SlewRateLimiter(1);
        yLimiter = new SlewRateLimiter(1);
        wLimiter = new SlewRateLimiter(1.4);
    }

    public enum ControlMode {
        FieldCentric,
        RobotCentric
    } 
}
