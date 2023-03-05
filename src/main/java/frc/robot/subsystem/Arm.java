package frc.robot.subsystem;
import frc.robot.component.hardware.SparkMaxComponent;
import frc.robot.component.hardware.TalonSRXComponent;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.SendableBuilder;

public class Arm extends SubsystemBase {
    private SparkMaxComponent tiltMotor;
    private TalonSRXComponent winchMotor;
    private SparkMaxPIDController tiltPIDController;
    private double targetTiltAngle;
    private static double targetWinchPosition;
    // private boolean winchGood;
    Position position;

    /**
     * Creates a new Arm from Constants
     */
    public Arm() {
        this.tiltMotor = new SparkMaxComponent(ArmConstants.TILT_MOTOR_ID, ArmConstants.TILT_MOTOR_TYPE);
        this.winchMotor = new TalonSRXComponent(ArmConstants.WINCH_MOTOR_ID);

        this.tiltMotor.setInverted(true);
        
        this.tiltPIDController = this.tiltMotor.getPIDController();
        this.tiltPIDController.setP(0.04);
        this.tiltPIDController.setI(0);
        this.tiltPIDController.setD(0);

        this.winchMotor.config_kP(0, 1.5);
        this.winchMotor.config_kI(0, 0);
        this.winchMotor.config_kD(0, 0);

        this.tiltPIDController.setOutputRange(-.5, .5);

        this.winchMotor.configAllowableClosedloopError(0, 400);
        this.winchMotor.configForwardSoftLimitEnable(true);
        this.winchMotor.configForwardSoftLimitThreshold(10000);
        this.winchMotor.configPeakOutputForward(.6);
        this.winchMotor.configPeakOutputReverse(-0.3);

    }
    
    /**
     * Creates a new Arm from Override
     * @param tiltMotor
     * @param winchMotor
     */
    public Arm(SparkMaxComponent tiltMotor, TalonSRXComponent winchMotor) {
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
        tiltMotor.setAngle(position);
    }

    /**
     * Sets the position of the winch motor. "position" is not speed, it is motor rotations
     * @param position is the angle that it has to turn
     */
    public void setWinch(double position) {
        targetWinchPosition = position;
        winchMotor.setAngle(position);
    }

    public void setOutput(double output) {
        tiltMotor.setOutput(output);
    }

    public void zero(){
        tiltMotor.getEncoder().setPosition(0);
    }
    /**
     * Command for setting the position of the tilt motor
     */
    public static class ArmSetTiltAngleCommand extends InstantCommand {
        private Arm arm;
        private double position;
        private boolean isLaunching;

        public ArmSetTiltAngleCommand(Arm arm, double position) {
            this.arm = arm;
            this.position = position;
            addRequirements(arm);
        }

        public ArmSetTiltAngleCommand(Arm arm, boolean isLaunching) {
            this.arm = arm;
            this.isLaunching = isLaunching;
        }
    
        public void initialize() {
            if (isLaunching) {
                arm.setTilt(ArmConstants.ARM_LAUNCH_ANGLE);
            } else {
                arm.setTilt(position);
            }
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

        // public boolean isFinished(){
        //     if (targetWinchPosition - arm.winchMotor.getAngle() < 400){
        //         return true;
        //     } 
        //     return false;
        // }


    }

    public double getWinchPosition() {
        return winchMotor.getSelectedSensorPosition() / 4096;
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
        builder.addDoubleProperty("Target tilt position", () -> targetTiltAngle, null);
        builder.addDoubleProperty("Target winch output", () -> targetWinchPosition, null);
    
        builder.addDoubleProperty("Current Encoder Tilt Position", () -> tiltMotor.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Current Winch Encoder Position", () -> winchMotor.getSelectedSensorPosition(), null);
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
