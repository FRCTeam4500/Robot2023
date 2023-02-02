package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.component.hardware.SparkMaxComponent;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.util.sendable.SendableBuilder;

public class Intake extends SubsystemBase {
    private SparkMaxComponent intakeMotor;
    private SparkMaxComponent intakeAngleMotor;
    private double targetTiltAngle;
    private double targetIntakeOutput;

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
        private double angle;

        public IntakeSetAngleCommand(Intake intake, double angle) {
            this.intake = intake;
            this.angle = angle;
        }

        @Override
        public void initialize() {
            intake.setAngle(angle);
        }
    }

    public class IntakeSetOutputCommand extends InstantCommand {
        private Intake intake;
        private double output;

        public IntakeSetOutputCommand(Intake intake, double output) {
            this.intake = intake;
            this.output = output;
        }

        @Override
        public void initialize() {
            intake.setIntake(output);
        }
    }

    public static Intake makeIntake() {
        return new Intake();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target Intake Tilt position", () -> targetTiltAngle, (value) -> {setAngle((double) value);});
        builder.addDoubleProperty("Target Intake Wheel output", () -> targetIntakeOutput, (value) -> {setIntake((double) value);});
    }
}