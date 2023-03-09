package frc.robot.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.DoubleSupplier;

public class DashboardNumberDisplay implements Sendable {
    DoubleSupplier number;
    String name;
    public DashboardNumberDisplay (String name, DoubleSupplier number){
        this.name = name;
        this.number = number;
    }

    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty(name,  number, null);
    }
}
