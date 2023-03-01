package frc.robot.subsystem;
import frc.robot.component.hardware.SparkMaxComponent;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.RobotContainer.isCone;
import static frc.robot.RobotContainer.isBottomCone;

public class Arm extends SubsystemBase {
    private SparkMaxComponent tiltMotor;
    private SparkMaxComponent winchMotor;
    private SparkMaxPIDController tiltPIDController;
    private SparkMaxPIDController winchPIDController;
    private double targetTiltAngle;
    private double targetWinchPosition;
    Position position;

    /**
     * Creates a new Arm from Constants
     */
    public Arm() {
        this.tiltMotor = new SparkMaxComponent(ArmConstants.TILT_MOTOR_ID, ArmConstants.TILT_MOTOR_TYPE);
        this.winchMotor = new SparkMaxComponent(ArmConstants.WINCH_MOTOR_ID, ArmConstants.WINCH_MOTOR_TYPE);

        this.tiltMotor.setInverted(true);
        
        this.tiltPIDController = this.tiltMotor.getPIDController();
        this.tiltPIDController.setP(0.04);
        this.tiltPIDController.setI(0);
        this.tiltPIDController.setD(0);

        this.winchPIDController = this.winchMotor.getPIDController();
        this.winchPIDController.setP(0.1);
        this.winchPIDController.setI(0);
        this.winchPIDController.setD(0);

        this.tiltPIDController.setOutputRange(-.3, .3);
        this.winchPIDController.setOutputRange(-.4, .4);
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
        tiltMotor.setAngle(position);
    }

    /**
     * Sets the position of the winch motor. "position" is not speed, it is motor rotations
     * @param position is the angle that it has to turn
     */
    public void setWinch(double position) {
        winchMotor.setAngle(position);
    }

    public void setOutput(double output) {
        winchMotor.setOutput(output);
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

        public ArmSetWinchOutputCommand(Arm arm, double output) {
            this.arm = arm;
            this.output = output;
            addRequirements(arm);
        }

        public void initialize() {
                arm.setWinch(output);
        }
    }

    public double getWinchPosition() {
        return winchMotor.getEncoder().getPosition();
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

    public static class ArmSetActualOutputCommand extends InstantCommand {
        private Arm arm;
        private double output;

        public ArmSetActualOutputCommand(Arm arm, double output) {
            this.arm = arm;
            this.output = output;
        }

        public void initialize() {
            arm.setOutput(output);
        }
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
    
        builder.addDoubleProperty("Current Encoder Tilt Position", () -> tiltMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Current Winch Encoder Position", () -> winchMotor.getEncoder().getPosition(), null);
        // builder.addStringProperty("Position", () -> {
        //     switch (position) {
        //         case Bottom:
        //             return "Bottom";
        //         case Middle:
        //             return "Middle";
        //         case Top:
        //             return "Top";
        //         case Retracted:
        //             return "Retracted";
        //         default:
        //             return "Unknown";
        //     }
        // }, null);
    }
}
