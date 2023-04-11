package frc.robot.subsystem.Swerve2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.component.hardware.AHRSAngleGetterComponent;

public class Swerve2 extends SubsystemBase{
    private SwerveDriveKinematics kinematics;
    private Swerve2Module[] modules;
    private SwerveModulePosition[] positions;
    private AHRSAngleGetterComponent gyro;
    private Translation2d[] translations;
    private SwerveModuleState[] states;
    private SwerveDriveOdometry odometry;
    private double currentGyroZero = 0.0;
    private ChassisSpeeds currentChassisSpeeds;
    private ChassisSpeeds targetChassisSpeeds;

    public Swerve2(AHRSAngleGetterComponent gyro, Swerve2Module... swerve2Modules) {
        modules = swerve2Modules; 
        this.gyro = gyro;
        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());
    }

    @Override
    public void periodic() {
        odometry.update(getGyroAngle(), getModulePositions());
        currentChassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldCentric(double forwardSpeed, double sidewaysSpeed, double turningSpeed) {
        driveRobotCentric(ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, sidewaysSpeed, turningSpeed, getFieldCentricGyroAngle()));
    }

    public void driveRobotCentric(double forwardSpeed, double sidewaysSpeed, double turningSpeed) {
        driveRobotCentric(new ChassisSpeeds(forwardSpeed, sidewaysSpeed, turningSpeed));
    }

    public void driveRobotCentric(ChassisSpeeds targetChassisSpeeds) {
        this.targetChassisSpeeds = targetChassisSpeeds;
        for(int i = 0; i < modules.length; i++) {
            modules[i].driveByChassisSpeeds(targetChassisSpeeds);
        }
    }

    public void resetPose(Pose2d newPose) {
        resetRobotAngle();
        odometry.resetPosition(getGyroAngle(), getModulePositions(), newPose);
    }

    public void resetRobotAngle() {
        resetRobotAngle(0);
    }

    public void resetRobotAngle(double angle) {
        currentGyroZero = getGyroAngle().getRadians() - angle;
    }

    public Pose2d getCurrentPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getGyroAngle() {
        return new Rotation2d(gyro.getAngle());
    }

    public Rotation2d getFieldCentricGyroAngle() {
        return new Rotation2d(getGyroAngle().getRadians() - currentGyroZero);
    }

    public Translation2d[] getModuleTranslations() {
        for(int i = 0; i < modules.length; i++) {
            translations[i] = modules[i].getTranslationFromCenter();
        }
        return translations;
    }

    public SwerveModulePosition[] getModulePositions() {
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Field Centric Gyro Angle: ", () -> (getFieldCentricGyroAngle().getRadians()), null);
        builder.addDoubleProperty("Robot Centric Gyro Angle: ", () -> (getGyroAngle().getRadians()), null);
        builder.addDoubleProperty("Target X Velocity: ", () -> (targetChassisSpeeds.vxMetersPerSecond), null);
        builder.addDoubleProperty("Target Y Velocity: ", () -> (targetChassisSpeeds.vyMetersPerSecond), null);
        builder.addDoubleProperty("Target W Velocity: ", () -> (targetChassisSpeeds.omegaRadiansPerSecond), null);
        builder.addDoubleProperty("Current X Velocity: ", () -> (currentChassisSpeeds.vxMetersPerSecond), null);
        builder.addDoubleProperty("Current Y Velocity: ", () -> (currentChassisSpeeds.vyMetersPerSecond), null);
        builder.addDoubleProperty("Current W Velocity: ", () -> (currentChassisSpeeds.omegaRadiansPerSecond), null);
        builder.addDoubleProperty("Odometric rotation: ", () -> getCurrentPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Odometric X: ", () -> getCurrentPose().getX(), null);
        builder.addDoubleProperty("Odometric Y: ", () -> getCurrentPose().getY(), null);
    }
}
