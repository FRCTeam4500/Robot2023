package frc.robot.subsystem.Swerve2;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.component.hardware.TalonFXComponent;
import static frc.robot.utility.ExtendedMath.ceiling;

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

    public void drive(double targetAngle, double targetVelocity) {
        double currentAngle = angleMotor.getAngle() *angleRotsPerMotorRots;

            
            double shortestRadianToTarget = getShortestRadianToTarget(currentAngle, state.angle.getRadians());
            double targetAngle = shortestRadianToTarget + currentAngle;
        angleMotor.setAngle(moduleTargetAngle / SwerveConstants.ANGLE_RATIO);
        angleMotor.setOutput(ceiling(moduleTargetVelocity / SwerveConstants.MAX_LINEAR_SPEED, 1) * 0.9); // I multiplied everything by 0.9 because I don't want the wheels going too fast
        
    }

    

    public Translation2d getTranslationFromCenter() {
        return translationFromCenter;
    }
}
