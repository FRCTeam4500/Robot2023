package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.component.hardware.SparkMaxComponent;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.util.sendable.SendableBuilder;

import static frc.robot.RobotContainer.isCone;
import static frc.robot.RobotContainer.isBottomCone;

public class Intake extends SubsystemBase {
    private SparkMaxComponent intakeMotor;
    private SparkMaxComponent intakeTiltMotor;
    private double targetTiltAngle;
    private double targetIntakeOutput;
    private SparkMaxPIDController tiltPIDController;
    private static boolean isCone2;

    public Intake() {
        this.intakeMotor = new SparkMaxComponent(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.INTAKE_MOTOR_TYPE);
        this.intakeTiltMotor = new SparkMaxComponent(IntakeConstants.INTAKE_ANGLE_MOTOR_ID, IntakeConstants.ANGLE_MOTOR_TYPE);

        this.intakeMotor.setIdleMode(IdleMode.kBrake);

        this.intakeTiltMotor.setIdleMode(IdleMode.kCoast);

        this.tiltPIDController = this.intakeTiltMotor.getPIDController();
        this.tiltPIDController.setP(1);
        this.tiltPIDController.setI(0);
        this.tiltPIDController.setD(0);

        this.tiltPIDController.setOutputRange(-.3, .3);
        this.intakeTiltMotor.setSoftLimit(SoftLimitDirection.kReverse, -40f);
    }

    public Intake(SparkMaxComponent intakeMotor, SparkMaxComponent intakeAngleMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeTiltMotor = intakeAngleMotor;
    }

    public void setIntake(double speed) {
        targetIntakeOutput = speed;
        intakeMotor.set(speed);
    }

    public void setAngle(double angle) {
        targetTiltAngle = angle;
        intakeTiltMotor.setAngle(angle);
    }

    public double getAngle() {
        return intakeTiltMotor.getEncoder().getPosition();
    }

    public void setAngleOutput(double output) {
        intakeTiltMotor.setOutput(output);
    }

    public static class IntakeSetAngleCommand extends InstantCommand {
        private Intake intake;
        private boolean launching;
        private boolean zeroing;
        private boolean placing;
        private boolean substation;

        
        public IntakeSetAngleCommand(Intake intake, boolean placing, boolean launching, boolean zeroing, boolean substation) {
            this.intake = intake;
            this.launching = launching;
            this.zeroing = zeroing;
            this.placing = placing;
            this.substation = substation;
        }

        
        

        @Override
        public void initialize() {
            if(substation){
                intake.setAngle(IntakeConstants.INTAKE_TRAY_PICKUP_ANGLE);
            } else if(launching) {
                intake.setAngle(IntakeConstants.INTAKE_LAUNCHING_ANGLE);
            } else if(zeroing){
                intake.setAngle(IntakeConstants.INTAKE_ZERO_ANGLE);
            }else if(!placing) {
                intake.setAngle(IntakeConstants.INTAKE_BOT_ANGLE);
            }else if(isBottomCone) {
                intake.setAngle(IntakeConstants.INTAKE_BOT_CONE_PLACE_ANGLE);
            }else {
                intake.setAngle(IntakeConstants.INTAKE_TOP_CONE_PLACE_ANGLE);
            }
        }
    }

    public static class IntakeSetOutputCommand extends InstantCommand {
        private Intake intake;
        private boolean placing;
        private boolean zeroing;

        public IntakeSetOutputCommand(Intake intake, boolean zeroing, boolean placing) {
            this.intake = intake;
            this.placing = placing;
            this.zeroing = zeroing;
        }

        @Override
        public void initialize() {
            if(zeroing) {
                intake.setIntake(0);
            } else if(placing) {
                if(isCone) {
                    intake.setIntake(IntakeConstants.INTAKE_CUBE_SPEED);
                } else {
                    intake.setIntake(IntakeConstants.INTAKE_CONE_SPEED);
                }
            } else {
                if(isCone) {
                    intake.setIntake(IntakeConstants.INTAKE_CONE_SPEED);
                } else {
                    intake.setIntake(IntakeConstants.INTAKE_CUBE_SPEED);
                }
            }
        }
    }

    public static class IntakeSetAngleOutputCommand extends CommandBase {
        Intake intake;
        double output;
        
        public IntakeSetAngleOutputCommand(Intake intake, double output) {
            this.intake = intake;
            this.output = output;
        }

        @Override
        public void initialize() {
            intake.setAngleOutput(output);
        }

    }

    public static Intake makeIntake() {
        return new Intake();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Intake Tilt position", () -> targetTiltAngle, (value) -> {setAngle((double) value);});
        builder.addDoubleProperty("Target Intake Wheel output", () -> targetIntakeOutput, (value) -> {setIntake((double) value);});
        builder.addBooleanProperty("IS CONE?!?!?", () -> isCone2, null);
        builder.addDoubleProperty("Intake Angle", () -> intakeTiltMotor.getEncoder().getPosition(), null);
    }
}
