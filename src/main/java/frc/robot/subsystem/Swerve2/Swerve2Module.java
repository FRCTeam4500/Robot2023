package frc.robot.subsystem.Swerve2;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.component.hardware.TalonFXComponent;
import static frc.robot.utility.ExtendedMath.ceiling;
import static frc.robot.utility.ExtendedMath.getShortestRadianToTarget;

public class Swerve2Module {
    private TalonFXComponent angleMotor;
    private TalonFXComponent driveMotor;
    private Translation2d translationFromCenter;
    private double moduleTargetAngle;
    private double moduleTargetVelocity;

    public Swerve2Module(TalonFXComponent angleMotor, TalonFXComponent driveMotor, Translation2d translationFromCenter) {
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.translationFromCenter = translationFromCenter;
    }

    public void drive(double initialTargetAngle, double initialTargetVelocity) {
        double targetAngle = initialTargetAngle;
        double oppositeAngle = targetAngle + Math.PI;
        double reverseTargetAngle = -Math.signum(targetAngle) * Math.abs(2*Math.PI - targetAngle);
        double reverseOppositeAngle = -Math.signum(oppositeAngle) * Math.abs(2*Math.PI - oppositeAngle);
        boolean reverseSpeed = Math.min(Math.abs(oppositeAngle), Math.abs(reverseOppositeAngle)) < Math.min(Math.abs(targetAngle), Math.abs(reverseTargetAngle));
        
        moduleTargetAngle = Math.min(targetAngle, Math.min(oppositeAngle, Math.min(reverseTargetAngle, reverseOppositeAngle)));
        moduleTargetVelocity = reverseSpeed ? -initialTargetVelocity : initialTargetVelocity;

        setModuleAngle(moduleTargetAngle);
        driveMotor.setOutput(ceiling(moduleTargetVelocity / SwerveConstants.MAX_LINEAR_SPEED, 1) * 0.9); // I multiplied everything by 0.9 because I don't want the wheels going too fast
    }


    public void setModuleAngle(double targetAngle) {
        angleMotor.setAngle(targetAngle / SwerveConstants.ANGLE_RATIO);
    }

    public double getModuleAngle() {
        return angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO;
    }

    public void setModuleVelocity() {

    }

    public double getModuleVelocity() {

    }



    public Translation2d getTranslationFromCenter() {
        return translationFromCenter;
    }
}
