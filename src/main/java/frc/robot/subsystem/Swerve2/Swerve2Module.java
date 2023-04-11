package frc.robot.subsystem.Swerve2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.component.hardware.TalonFXComponent;
import static frc.robot.utility.ExtendedMath.ceiling;
import static frc.robot.utility.ExtendedMath.closestToZero;

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

    public void driveByChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
        double targetForwardVelocity = targetChassisSpeeds.vxMetersPerSecond;
        double targetSidewaysVelocity = targetChassisSpeeds.vyMetersPerSecond;
        double targetRotationalVelocity = targetChassisSpeeds.omegaRadiansPerSecond;

        double xVector = targetForwardVelocity - targetRotationalVelocity * translationFromCenter.getY();
        double yVector = targetSidewaysVelocity + targetRotationalVelocity * translationFromCenter.getX();
        double rawVelocity = Math.sqrt(Math.pow(xVector, 2) + Math.pow(yVector, 2));
        double rawAngle = Math.atan2(yVector, xVector);

        driveOptimized(rawAngle, rawVelocity);
    }

    public void driveOptimized(double initialTargetAngle, double initialTargetVelocity) {
        double currentAngle = getModuleAngle();
        double targetAngle = initialTargetAngle - currentAngle;
        double oppositeAngle = targetAngle + Math.PI;
        double reverseTargetAngle = -Math.signum(targetAngle) * Math.abs(2*Math.PI - targetAngle);
        double reverseOppositeAngle = -Math.signum(oppositeAngle) * Math.abs(2*Math.PI - oppositeAngle);
        boolean reverseSpeed = Math.min(Math.abs(oppositeAngle), Math.abs(reverseOppositeAngle)) < Math.min(Math.abs(targetAngle), Math.abs(reverseTargetAngle));
        moduleTargetAngle = closestToZero(targetAngle, oppositeAngle, reverseTargetAngle, reverseTargetAngle) + currentAngle;
        moduleTargetVelocity = reverseSpeed ? -initialTargetVelocity : initialTargetVelocity;

        setModuleAngle(moduleTargetAngle);
        setModuleVelocity(moduleTargetVelocity); 
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleVelocity(), new Rotation2d(getModuleAngle()));
    }

    public double getModuleTargetAngle() {
        return moduleTargetAngle;
    }

    public double getModuleTargetVelocity() {
        return moduleTargetVelocity;
    }

    public void setModuleAngle(double targetAngle) {
        angleMotor.setAngle(targetAngle / SwerveConstants.ANGLE_RATIO);
    }

    public double getModuleAngle() {
        return angleMotor.getAngle() * SwerveConstants.ANGLE_RATIO;
    }

    public void setModuleVelocity(double targetVelocity) {
        driveMotor.setVelocity(ceiling(targetVelocity, SwerveConstants.MAX_LINEAR_SPEED));
    }

    public double getModuleVelocity() {
        return driveMotor.getVelocity();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getPoseMeters(), new Rotation2d(getModuleAngle()));
    }

    public Translation2d getTranslationFromCenter() {
        return translationFromCenter;
    }
}
