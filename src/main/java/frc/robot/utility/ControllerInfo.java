package frc.robot.utility;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A class representing controller configuration values
 * It is essentially a struct, so the members are all public
 * Its a reference type so it can be passed once then its members can be changed on the fly
 * Its not super standard java but I don't see anything wrong with it
 */

public class ControllerInfo implements Sendable {
    public double xDeadzone = 0;
    public double yDeadzone = 0;
    public double zDeadzone = 0;
    public double xSensitivity = 0;
    public double ySensitivity = 0;
    public double zSensitivity = 0;

    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("X Axis Sensitivity", () -> {return xSensitivity;}, (value) -> {xSensitivity = value;});
        builder.addDoubleProperty("Y Axis Sensitivity", () -> {return ySensitivity;}, (value) -> {ySensitivity = value;});
        builder.addDoubleProperty("Z Axis Sensitivity", () -> {return zSensitivity;}, (value) -> {zSensitivity = value;});
        builder.addDoubleProperty("X Axis Deadzone", () -> {return xDeadzone;}, (value) -> {xDeadzone = value;});
        builder.addDoubleProperty("Y Axis Deadzone", () -> {return yDeadzone;}, (value) -> {yDeadzone = value;});
        builder.addDoubleProperty("Z Axis Deadzone", () -> {return zDeadzone;}, (value) -> {zDeadzone = value;});
    }
}
