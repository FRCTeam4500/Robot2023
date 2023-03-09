package frc.robot.dashboard;


import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class DashboardNumberSetting {
    DoubleSupplier number;
    DoubleConsumer setting;
    String name;

    public DashboardNumberSetting (String name, DoubleSupplier number, DoubleConsumer setting){
        this.name = name;
        this.number = number;
        this.setting = setting;
    }

    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty(name, number, setting);
    }
}
