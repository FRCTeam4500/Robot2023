package frc.robot.subsystem.swerve.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.dashboard.DashboardMessageDisplay;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.ControllerInfo;
import edu.wpi.first.util.sendable.Sendable;

/**
 * A swerve command with support for three swerve control modes:
 *
 * Field-Centric:
 * Robot moves relative to the field's axes.
 * When pushing the joystick forward, the robot moves down the field, no matter which way it is facing
 * (Actually, it moves in whatever direction is zeroed to, this just assumes that the gyro is zeroed down the field)
 *
 * Robot-Centric
 * The robot moves relative to itself.
 * When pushing the joystick foward, the robot moves in whatever direction it is facing.
 * For our purposes, the front of the robot is the intake side.
 *
 * AlignToAngle
 * Aligns to a target angle
 * TODO: make this automatically align the robot rotationally with the goal, to avoid the goal going out of view of the vision
 */
public class TriModeSwerveCommand extends CommandBase implements Sendable {
    private Swerve swerve;
    private Joystick joystick;
    private ControllerInfo info;
    private DashboardMessageDisplay messageDisplay;

    private PIDController angleAdjustmentController;
    public ControlMode controlMode;

    public SlewRateLimiter xLimiter = new SlewRateLimiter(1);
    public SlewRateLimiter yLimiter = new SlewRateLimiter(1);
    public SlewRateLimiter zLimiter = new SlewRateLimiter(1);

    public boolean lockRotation = false;
    public boolean limitSpeed = false;
    public boolean noForward = false;
    public double targetAngle = 0;

    private double limitedSpeed = .2;  // was .75


    public TriModeSwerveCommand(Swerve swerve, Joystick joystick, ControllerInfo controllerInfo, DashboardMessageDisplay messageDisplay){
        this.swerve = swerve;
        this.joystick = joystick;
        info = controllerInfo;
        this.messageDisplay = messageDisplay;
        controlMode = ControlMode.FieldCentric; //default control mode is field-centric
        angleAdjustmentController = new PIDController(1,0,0);
        angleAdjustmentController.enableContinuousInput(-Math.PI, Math.PI);

        

        addRequirements(swerve);
    }

    @Override
    public void execute(){
        double xSpeed = -xLimiter.calculate(joystick.getX()) * info.xSensitivity;
        double ySpeed = -yLimiter.calculate(joystick.getY()) * info.ySensitivity;
        double zSpeed = -zLimiter.calculate(joystick.getZ()) * info.zSensitivity;
        if (limitSpeed){
            xSpeed = limitedSpeed * xSpeed; //ceiling(xSpeed, limitedSpeed);
            ySpeed = limitedSpeed * ySpeed; //ceiling(ySpeed, limitedSpeed);
            zSpeed = limitedSpeed * zSpeed; //ceiling(zSpeed, limitedSpeed);
        }
        if (lockRotation) {
            zSpeed = 0;
        }
        if (noForward) {
            ySpeed = 0;
            zSpeed = 0;
            xSpeed = ceiling(xSpeed, limitedSpeed);
        }        
        switch (controlMode){
            case FieldCentric:
                moveFieldCentric(xSpeed, ySpeed, zSpeed);
                break;
            case RobotCentric:
                moveRobotCentric(xSpeed,ySpeed,zSpeed);
                break;
            case AlignToAngle:
                moveAlign(xSpeed,ySpeed,zSpeed);
                break;
        }
    }

    private void moveFieldCentric(double x, double y, double w){
        swerve.moveFieldCentric(y,x,w);
    }
    private void moveRobotCentric(double x, double y, double w){
        swerve.moveRobotCentric(y,x,w);
    }
    private void moveAlign(double r, double t, double w) {
        double wSpeed = 4 * angleAdjustmentController.calculate(swerve.getRobotAngle(), targetAngle);
        moveFieldCentric(r, t, wSpeed);
    }
    //deadzones the input
    private double withDeadzone(double value, double deadzone){
        if(Math.abs(value) < deadzone)
            return 0;
        else
            return value;
    }

    public enum ControlMode{
        FieldCentric,
        RobotCentric,
        AlignToAngle
    }

    private double ceiling(double value, double maximum){
        if (Math.abs(value) > maximum){
            return maximum * Math.signum(value);
        }
        return value;
    }

    public void initSendable(SendableBuilder builder){
        builder.addStringProperty("Drive Mode", () -> {
            switch (controlMode) {
                case FieldCentric:
                    return "Field Centric";
            
                case RobotCentric:
                    return "Robot Centric";

                case AlignToAngle:
                    return "Align To Angle";
            }
            return "";
        }, null);
        builder.addDoubleProperty("controller x", joystick::getX, null);
        builder.addDoubleProperty("controller y", joystick::getY, null);
        builder.addDoubleProperty("controller z", joystick::getZ, null);
        builder.addBooleanProperty("Limit Speed", () -> {return limitSpeed;}, null);
        
    }
}
