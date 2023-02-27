package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.component.hardware.SparkMaxComponent;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.util.sendable.SendableBuilder;

public class Intake extends SubsystemBase {
    private SparkMaxComponent intakeMotor;
    public SparkMaxComponent intakeAngleMotor;
    private double targetTiltAngle;
    private double targetIntakeOutput;
    private static boolean isCone2;

    public Intake() {
        this.intakeMotor = new SparkMaxComponent(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.INTAKE_MOTOR_TYPE);
        this.intakeAngleMotor = new SparkMaxComponent(IntakeConstants.INTAKE_ANGLE_MOTOR_ID, IntakeConstants.ANGLE_MOTOR_TYPE);
    }

    public Intake(SparkMaxComponent intakeMotor, SparkMaxComponent intakeAngleMotor) {
        this.intakeMotor = intakeMotor;
        this.intakeAngleMotor = intakeAngleMotor;
    }

    public void setIntake(double speed) {
        targetIntakeOutput = speed;
        intakeMotor.set(speed);
    }

    public void setAngle(double angle) {
        targetTiltAngle = angle;
        intakeAngleMotor.setAngle(angle);
    }

    public static class IntakeSetAngleCommand extends InstantCommand {
        private Intake intake;
        private boolean isCone;
        private boolean isBottomCone;
        private boolean zeroing;

        public IntakeSetAngleCommand(Intake intake, boolean isCone, boolean isBottomCone, boolean zeroing) {
            this.intake = intake;
            this.isCone = isCone;
            this.isBottomCone = isBottomCone;
            this.zeroing = zeroing;
        }

        @Override
        public void initialize() {
            if(zeroing){
                intake.setAngle(IntakeConstants.INTAKE_RETRACTED_ANGLE);
            } else {
                if(isCone){
                    if(isBottomCone){
                        intake.setAngle(IntakeConstants.INTAKE_BOT_ANGLE);
                    } else {
                        intake.setAngle(IntakeConstants.INTAKE_TOP_CONE_PLACE_ANGLE);
                    }
                } else {
                    intake.setAngle(IntakeConstants.INTAKE_CUBE_PLACE_ANGLE);
                }
            }
        }
    }

    public static class IntakeSetOutputCommand extends InstantCommand {
        private Intake intake;
        private boolean isCone;
        private boolean zeroing;

        public IntakeSetOutputCommand(Intake intake, boolean isCone, boolean zeroing) {
            this.intake = intake;
            this.isCone = isCone;
            this.zeroing = zeroing;
        }

        @Override
        public void initialize() {
            isCone2 = isCone;
            if(zeroing){
                intake.setIntake(0);
            } else {
                if(isCone){
                    intake.setIntake(IntakeConstants.INTAKE_CONE_SPEED);
                } else {
                    intake.setIntake(IntakeConstants.INTAKE_CUBE_SPEED);
                }
            }
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
    }
}
