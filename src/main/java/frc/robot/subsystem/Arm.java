package frc.robot.subsystem;
import frc.robot.component.hardware.SparkMaxComponent;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
     * Sets the position of the winch motor. "Output" is not speed, it is angle
     * @param output is the angle that it has to turn
     */
    public void setWinch(double output) {
        targetWinchOutput = output;
        winchMotor.set(output);
    }

    /**
     * Command for setting the position of the tilt motor
     */
    public static class ArmSetTiltAngleCommand extends InstantCommand {
        private Arm arm;
        private double position;

        public ArmSetTiltAngleCommand(Arm arm, double position) {
            this.arm = arm;
            this.position = position;
            addRequirements(arm);
        }
    
        public void initialize() {
            arm.setTilt(position);
        }
    }

    public static class ArmSetWinchOutputCommand extends InstantCommand {
        private Arm arm;
        private double output;
        private boolean isBottomCone;

        public ArmSetWinchOutputCommand(Arm arm, double output, boolean isBottomCone) {
            this.arm = arm;
            this.output = output;
            this.isBottomCone = isBottomCone;
            addRequirements(arm);
        }

        public void initialize() {
                if(isBottomCone && output!=ArmConstants.ARM_RETRACT){
                    arm.setWinch(output + ArmConstants.ARM_BOT_CONE_ADDITION);
                } else {
                    arm.setWinch(output);
                }
        }
    }

    /** 
     * Positions for the Arm (Synchronize tilt and winch)
    */
    public enum Position {
        Bottom,
        Middle,
        Top,
        Retracted,
        Ground
    }

    public Position getPosition() {
        return position;
    }

    public static Arm makeArm() {
        return new Arm();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target tilt position", () -> targetTiltAngle, (value) -> {setTilt((double) value);});
        builder.addDoubleProperty("Target winch output", () -> targetWinchOutput, (value) -> {setWinch((double) value);});
    }
}
