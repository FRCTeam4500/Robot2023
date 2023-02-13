package frc.robot.subsystem.swerve.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.component.hardware.AHRSAngleGetterComponent;
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
 * TODO: make this automatically align the robot rotationally with the goal,
 * to avoid the goal going out of view of the vision
 */
public class TriModeSwerveCommand extends CommandBase {
    private Swerve swerve;
    private Joystick joystick;
    private ControllerInfo info;
    private DashboardMessageDisplay messageDisplay;

    private PIDController angleAdjustmentController;
    public ControlMode controlMode;

    public boolean lockRotation = false;
    public boolean limitSpeed = false;
    public double targetAngle = 0;
    public boolean noForward = false;
    public boolean balance = false;

    private double limitedSpeed = .75;


    public TriModeSwerveCommand(Swerve swerve, Joystick joystick, ControllerInfo controllerInfo, DashboardMessageDisplay messageDisplay) {
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
        double xSpeed = -withDeadzone(joystick.getX(), info.xDeadzone) * info.xSensitivity;
        double ySpeed = -withDeadzone(joystick.getY(), info.yDeadzone) * info.ySensitivity;
        double zSpeed = -withDeadzone(joystick.getZ(), info.zDeadzone) * info.zSensitivity;
        if (limitSpeed){
            xSpeed = ceiling(xSpeed, limitedSpeed);
            ySpeed = ceiling(ySpeed, limitedSpeed);
            zSpeed = ceiling(zSpeed, limitedSpeed);
        }
        if (lockRotation)
            zSpeed = 0;
        if (noForward) {
            ySpeed = 0;
            zSpeed = 0;
            xSpeed = ceiling(xSpeed, limitedSpeed);
        }
        if (balance) {
            balance();
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
    private void balance() {
        AHRSAngleGetterComponent gyro = new AHRSAngleGetterComponent(I2C.Port.kMXP);
        double anglePitch;
        double angleRoll;
        int timesEqualRoll = 0;
        int timesEqualPitch = 0;

        while (timesEqualRoll < 10 || timesEqualPitch < 10) {
            anglePitch = Math.toDegrees(gyro.getPitch());
            angleRoll = Math.toDegrees(gyro.getRoll());
            if (false) { // Nate code
                if (-2 < anglePitch && anglePitch < 2) {
                    timesEqualPitch++;
                }
                if (-2 < angleRoll && angleRoll < 2) {
                    timesEqualRoll++;
                }
                moveRobotCentric(-anglePitch/30, -angleRoll/30, 0);
            } else { // Vincent Bryan code

                if (anglePitch > 2) {
                    timesEqualPitch = 0;
                    moveRobotCentric((anglePitch - 2) / 10, 0, 0);
                } else if (anglePitch < -2) {
                    timesEqualPitch = 0;
                    moveRobotCentric(-((anglePitch + 2) / 10), 0, 0);
                } else {
                    timesEqualPitch++;
                }
                if (angleRoll > 2) {
                    timesEqualRoll = 0;
                    moveRobotCentric(0, (angleRoll - 2) / 10, 0);
                } else if (angleRoll < -2) {
                    timesEqualRoll = 0;
                    moveRobotCentric(0, (angleRoll - 2) / 10, 0);
                } else {
                    timesEqualRoll++;
                }
            }
            new WaitCommand(.05); // Stops continuous running
        }
    } //float tilt = ((gyro.getPitch()*gyro.getY())+(gyro.getRoll()*gyro.getX()))/(gyro.getX()*gyro.getY())
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
