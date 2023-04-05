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
    private SwerveDriveOdometry odometry;
    private double currentGyroZero = 0.0;

    public Swerve2(AHRSAngleGetterComponent gyro, Swerve2Module... swerve2Modules) {
        modules = swerve2Modules; 
        this.gyro = gyro;
        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());
    }

    @Override
    public void periodic() {
        odometry.update(getGyroAngle(), getModulePositions());
    }

    public void driveFieldCentric(double forwardSpeed, double sidewaysSpeed, double turningSpeed) {
        driveRobotCentric(ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, sidewaysSpeed, turningSpeed, new Rotation2d(getGyroAngle().getRadians() - currentGyroZero)));
    }

    public void driveRobotCentric(double forwardSpeed, double sidewaysSpeed, double turningSpeed) {
        driveRobotCentric(new ChassisSpeeds(forwardSpeed, sidewaysSpeed, turningSpeed));
    }

    public void driveRobotCentric(ChassisSpeeds targetChassisSpeeds) {
        driveModules(kinematics.toSwerveModuleStates(targetChassisSpeeds));
    }

    public void driveModules(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.MAX_LINEAR_SPEED);

        for (int i = 0; i < modules.length; i++) {
            modules[i].driveByState(targetStates[i]);
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

    public void initSendable(SendableBuilder builder) {
        
    }
}
