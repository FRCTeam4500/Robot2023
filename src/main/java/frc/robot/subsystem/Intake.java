package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.component.hardware.SparkMaxComponent;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
        private double angle;

        /**Use this constructor to do everything except place 
         * @param intake The intake whose angle should be set
         * @param angle The angle the intake should be set to
         */
        public IntakeSetAngleCommand(Intake intake, double angle) {
            this.intake = intake;
            this.angle = angle;
        }

        /**
         * Use this constructor to place game pieces 
         * @param intake The intake whose angle should be set
         */
        public IntakeSetAngleCommand(Intake intake) {
            this(intake, isBottomCone ? IntakeConstants.INTAKE_BOT_CONE_PLACE_ANGLE : IntakeConstants.INTAKE_TOP_CONE_PLACE_ANGLE);
        }

        @Override
        public void initialize() {
            intake.setAngle(angle);
        }

    }

    public static class IntakeSetOutputCommand extends InstantCommand {
        private Intake intake;
        private double speed;

        /**
         * Only use this constructor to zero the intake speed.
         * @param intake The intake whose output should be set
         */
        public IntakeSetOutputCommand(Intake intake) {
            this.intake = intake;
            this.speed = 0;
        }
        /**
         * 
         * @param intake The intake whose output should be set
         * @param placing If we're placing, this should be true. If we're picking up, this should be false.
         */
        public IntakeSetOutputCommand(Intake intake, boolean placing) {
            this.intake = intake;
            if((placing && isCone) || (!placing && !isCone)) { // If we're placing a cone or picking up a cube
                this.speed = IntakeConstants.INTAKE_CUBE_SPEED;
            } else if ((placing && !isCone) || (!placing && isCone)) { // If we're placing a cube or picking up a cone
                this.speed = IntakeConstants.INTAKE_CONE_SPEED;
            }
            
            
            
        }

        @Override
        public void initialize() {
            intake.setIntake(speed);
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
        builder.addDoubleProperty("Target Intake Tilt angle", () -> targetTiltAngle, (value) -> {setAngle((double) value);});
        builder.addDoubleProperty("Target Intake Wheel output", () -> targetIntakeOutput, (value) -> {setIntake((double) value);});
        builder.addBooleanProperty("IS CONE?!?!?", () -> isCone, null);
        builder.addDoubleProperty("Intake Angle", () -> intakeTiltMotor.getEncoder().getPosition(), null);
    }
}
