package frc.robot.component.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.component.OutputComponent;

public class VictorSPXComponent extends VictorSPX implements OutputComponent {

    public VictorSPXComponent(int port){
        super(port);
    }

    public void setOutput(double output){
        set(ControlMode.PercentOutput, output);
    }

    @Override
    public double getOutput() {
        //return getMotorOutputPercent();
        return 0;
    }
}
