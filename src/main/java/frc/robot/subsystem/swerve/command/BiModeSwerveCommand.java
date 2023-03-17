package frc.robot.subsystem.swerve.command;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.Swerve;

/**
 * A swerve command with support for two swerve control modes:
 * <p>
 * Field-Centric:
 * Robot moves relative to the field's axes.
 * When pushing the joystick forward, the robot moves down the field, no matter which way it is facing
 * (Actually, it moves in whatever direction is zeroed to, this just assumes that the gyro is zeroed down the field)
 * <p>
 * Robot-Centric:
 * The robot moves relative to itself.
 * When pushing the joystick foward, the robot moves in whatever direction it is facing.
 * For our purposes, the front of the robot is the intake side.
 */
public class BiModeSwerveCommand extends CommandBase {
    private Swerve swerve;
    private CommandXboxController controller;

    public ControlMode controlMode;

    public SlewRateLimiter xLimiter = new SlewRateLimiter(1);
    public SlewRateLimiter yLimiter = new SlewRateLimiter(1);
    public SlewRateLimiter zLimiter = new SlewRateLimiter(1.4);
    private double xSens;
    private double ySens;
    private double zSens;

    public double targetAngle = 0;


    public BiModeSwerveCommand(Swerve swerve, CommandXboxController controller){
        this.swerve = swerve;
        this.controller = controller;
        controlMode = ControlMode.FieldCentric; //default control mode is field-centric
        midSpeed();
        addRequirements(swerve);
    }

    @Override
    public void execute(){
        double xSpeed = -xLimiter.calculate(controller.getLeftX()) * xSens;
        double ySpeed = -yLimiter.calculate(controller.getLeftY()) * ySens;
        double zSpeed = -zLimiter.calculate(controller.getRightX()) * zSens;
        switch (controlMode){
            case FieldCentric:
                moveFieldCentric(xSpeed, ySpeed, zSpeed);
                break;
            case RobotCentric:
                moveRobotCentric(xSpeed,ySpeed,zSpeed);
                break;
        }
    }

    private void moveFieldCentric(double x, double y, double w){
        swerve.moveFieldCentric(y,x,w);
    }
    private void moveRobotCentric(double x, double y, double w){
        swerve.moveRobotCentric(y,x,w);
    }

    public enum ControlMode{
        FieldCentric,
        RobotCentric
    }

    public void fastSpeed() {
        xSens = 4;
        ySens = 4;
        zSens = 3.5;
    }

    public void midSpeed() {
        xSens = 3;
        ySens = 3;
        zSens = 2.5;
    }

    public void slowSpeed() {
        xSens = 1;
        ySens = 1;
        zSens = 1;
    }

    /**
     * Switches between RobotCentric and FieldCentric
     */
    public void switchControlMode() {
        if (controlMode == ControlMode.RobotCentric){
            controlMode = ControlMode.FieldCentric;
        } else {
            controlMode = ControlMode.RobotCentric;
        }
    }

    public void initSendable(SendableBuilder builder){
        builder.addStringProperty("Drive Mode", () -> {return controlMode.name();}, null);
        
    }
}
