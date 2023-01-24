
package frc.robot.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DashboardBooleanDisplay implements Sendable {
    BooleanSupplier bool;
    String name;
    public DashboardBooleanDisplay (String name, BooleanSupplier boolr){
        this.name = name;
        this.bool = boolr;
    }

    public void initSendable(SendableBuilder builder){
        builder.addBooleanProperty(name,  bool, null);
    }
}
