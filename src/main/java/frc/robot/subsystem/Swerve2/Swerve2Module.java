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
        double workingTargetAngle = initialTargetAngle;
        
        // This line figures out what way the wheel should turn to get to the target angle fastest.
        workingTargetAngle = getShortestRadianToTarget(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO, workingTargetAngle) + angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO;
        
        // This if statement determines if it is faster to 
        // turn to the opposite of the target angle and reverse the wheel speed
        // rather than turning to the actual target angle
        if(
            Math.abs(getShortestRadianToTarget(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO, workingTargetAngle + Math.PI)) 
            < Math.abs(getShortestRadianToTarget(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO, workingTargetAngle))
        ) {
            moduleTargetAngle = angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO + getShortestRadianToTarget(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO, workingTargetAngle + Math.PI);
            moduleTargetVelocity = -initialTargetVelocity;
        } else {
            moduleTargetAngle = angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO + getShortestRadianToTarget(angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO, workingTargetAngle);
            moduleTargetVelocity = initialTargetVelocity;
        }

        angleMotor.setAngle(moduleTargetAngle / SwerveConstants.ANGLE_RATIO);
        driveMotor.setOutput(ceiling(moduleTargetVelocity / SwerveConstants.MAX_LINEAR_SPEED, 1) * 0.9); // I multiplied everything by 0.9 because I don't want the wheels going too fast
    }

    public double getVelocity() {
        return driveMotor.getAngularVelocity();
    }

    /**
     * Sets the velocity of a swerve module. Not recommened to use directly. 
     * @param targetVelocity The speed
     */
    public void setVelocity(double targetVelocity) {
        driveMotor.setOutput(ceiling(targetVelocity / SwerveConstants.MAX_LINEAR_SPEED, 1) * 0.9);
    }

    public double getAngle() {
        return angleMotor.getAngle();
    }

    public void coast() {
        driveMotor.setOutput(0);
    }


    public Translation2d getTranslationFromCenter() {
        return translationFromCenter;
    }
}
