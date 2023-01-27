package frc.robot.subsystem;
import frc.robot.component.hardware.SparkMaxComponent;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Arm extends SubsystemBase {
    private SparkMaxComponent tiltMotor;
    private SparkMaxComponent winchMotor;
    private double targetTiltAngle;
    private double targetWinchOutput;
    Position position;

    /**
     * Creates a new Arm from Constants
     */
    public Arm() {
        this.tiltMotor = new SparkMaxComponent(ArmConstants.TILT_MOTOR_ID, ArmConstants.TILT_MOTOR_TYPE);
        this.winchMotor = new SparkMaxComponent(ArmConstants.WINCH_MOTOR_ID, ArmConstants.WINCH_MOTOR_TYPE);
    }
    
    /**
     * Creates a new Arm from Override
     * @param tiltMotor
     * @param winchMotor
     */
    public Arm(SparkMaxComponent tiltMotor, SparkMaxComponent winchMotor) {
        this.tiltMotor = tiltMotor;
        this.winchMotor = winchMotor;
    }

    /**
     * Sets the position of the tilt motor
     * @param position
     */
    public void setTilt(double position) {
        targetTiltAngle = position;
        tiltMotor.setAngle(position);
    }

    /**
     * Sets the output of the winch motor
     * @param output
     */
    public void setWinch(double output) {
        targetWinchOutput = output;
        winchMotor.set(output);
    }

    /**
     * Command for setting the position of the tilt motor
     */
    public class ArmSetTiltAngleCommand extends InstantCommand {
        private Arm arm;
        private int position;

        public ArmSetTiltAngleCommand(Arm arm, int position) {
            this.arm = arm;
            this.position = position;
            addRequirements(arm);
        }
    
        public void initialize() {
            arm.setTilt(position);
        }
    }

    public class ArmSetWinchOutputCommand extends InstantCommand {
        private Arm arm;
        private double output;

        public ArmSetWinchOutputCommand(Arm arm, double output) {
            this.arm = arm;
            this.output = output;
            addRequirements(arm);
        }

        public void initialize() {
            arm.setWinch(output);
        }
    }

    /** 
     * Positions for the Arm (Synchronize tilt and winch)
     * @param Bottom
     * @param Middle
     * @param Top
     * @param Retracted
    */
    public enum Position {
        Bottom,
        Middle,
        Top,
        Retracted
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target tilt position", () -> targetTiltAngle, (value) -> {setTilt((double) value);});
        builder.addDoubleProperty("Target winch output", () -> targetWinchOutput, (value) ->{setWinch((double) value);});

        builder.addStringProperty("Position", () -> {
            switch (position) {
                case Bottom:
                    return "Bottom";
            
                case Middle:
                    return "Middle";

                case Top:
                    return "Top";
                case Retracted:
                    return "Retracted";
            }
            return "";
        }, null);
    }
}