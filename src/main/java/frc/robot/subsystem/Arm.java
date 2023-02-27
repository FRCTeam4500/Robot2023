package frc.robot.subsystem;
import frc.robot.component.hardware.SparkMaxComponent;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Arm extends SubsystemBase {
    private SparkMaxComponent tiltMotor;
    private SparkMaxComponent winchMotor;
    private RelativeEncoder tiltEncoder;
    private RelativeEncoder winchEncoder;
    private double targetTiltAngle;
    private double targetWinchPosition;
    Position position;

    /**
     * Creates a new Arm from Constants
     */
    public Arm() {
        this.tiltMotor = new SparkMaxComponent(ArmConstants.TILT_MOTOR_ID, ArmConstants.TILT_MOTOR_TYPE);
        this.winchMotor = new SparkMaxComponent(ArmConstants.WINCH_MOTOR_ID, ArmConstants.WINCH_MOTOR_TYPE);
        
        this.tiltEncoder = tiltMotor.getEncoder();
        this.winchEncoder = winchMotor.getEncoder();

        this.tiltMotor.setSoftLimit(SoftLimitDirection.kForward, 16f); // The "f" is specifying that it is a float
        this.tiltMotor.setSoftLimit(SoftLimitDirection.kReverse, 0f);
        this.winchMotor.setSoftLimit(SoftLimitDirection.kForward, 55f);
        this.winchMotor.setSoftLimit(SoftLimitDirection.kReverse, 0.5f);

        this.tiltMotor.setIdleMode(IdleMode.kBrake);
        this.winchMotor.setIdleMode(IdleMode.kBrake);
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
     * Sets the position of the tilt motor.
     * 
     * Depending on if we are going up or down, it will set the motor to go up or down,
     * The "? :" is a ternary operator, it is the same as "if else".
     * 
     * @param position
     */
    public void setTilt(double position) {
        targetTiltAngle = position;
        boolean up = targetTiltAngle > tiltEncoder.getPosition();

        while (up ? tiltEncoder.getPosition() < targetTiltAngle :
                tiltEncoder.getPosition() > targetTiltAngle) {
            tiltMotor.setOutput(up ? .5 : -.5);
        }
    }

    /**
     * Sets the position of the winch motor. "position" is not speed, it is motor rotations
     * @param position is the angle that it has to turn
     */
    public void setWinch(double position) {
        targetWinchPosition = position;
        boolean forward = targetWinchPosition > winchEncoder.getPosition();

        while (forward ? winchEncoder.getPosition() < targetWinchPosition :
                winchEncoder.getPosition() > targetWinchPosition) {
            winchMotor.setOutput(forward ? .5 : -.5);
        }
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
                if(isBottomCone && output!=ArmConstants.ARM_RETRACT) {
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
        Retracted
    }

    public Position getPosition() {
        return position;
    }

    public void setPosition(Position position) {
        this.position = position;
    }

    public static Arm makeArm() {
        return new Arm();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target tilt position", () -> targetTiltAngle, (value) -> {setTilt((double) value);});
        builder.addDoubleProperty("Target winch output", () -> targetWinchPosition, (value) -> {setWinch((double) value);});
    
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
                default:
                    return "Unknown";
            }
        }, null);
    }
}
