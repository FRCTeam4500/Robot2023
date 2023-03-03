package frc.robot.subsystem.swerve.command;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.ControllerInfo;
import frc.robot.utility.ExtendedMath;

/**
 * Joystick movement :)
 */
public class SwerveDefaultCommand extends CommandBase implements Sendable {
    private Swerve swerve;
    private Joystick joystick;
    private ControllerInfo info;
    public boolean isRobotCentric;
    public boolean lockRotation = false;

    public SwerveDefaultCommand(Swerve swerve, Joystick joystick, ControllerInfo info){
        this.swerve = swerve;
        this.joystick = joystick;
        this.info = info;
        addRequirements(swerve);
    }

    public void execute(){
        double xSpeed = ExtendedMath.withHardDeadzone(joystick.getX(), info.xDeadzone);
        double ySpeed = ExtendedMath.withHardDeadzone(-joystick.getY(), info.yDeadzone);
        double zSpeed = lockRotation ? 0 : ExtendedMath.withHardDeadzone(-joystick.getZ(), info.zDeadzone);
        if (isRobotCentric){
            swerve.moveRobotCentric(
                    ySpeed * info.ySensitivity,
                    xSpeed * info.xSensitivity,
                    zSpeed * info.zSensitivity
            );
            return;
        }
        swerve.moveFieldCentric(
                ySpeed * info.ySensitivity,
                xSpeed * info.xSensitivity,
                zSpeed * info.zSensitivity
        );
    }

     @Override
     public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Controller X", joystick::getX, null);
        builder.addDoubleProperty("Controller Y", joystick::getY, null);
        builder.addDoubleProperty("Controller Z", joystick::getZ, null);
     }
 }
