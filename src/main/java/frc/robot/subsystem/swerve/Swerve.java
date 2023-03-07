package frc.robot.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.component.hardware.AHRSAngleGetterComponent;

public interface Swerve extends Subsystem, Sendable {
    void moveRobotCentric( double ySpeed, double xSpeed, double wSpeed );
    void moveRobotCentric(ChassisSpeeds chassisSpeeds);
    void moveRobotCentric(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation);
    void moveAngleCentric(double ySpeed, double xSpeed, double wSpeed, Rotation2d robotAngle);
    void moveFieldCentric(ChassisSpeeds chassisSpeeds);
    void moveFieldCentric(double xSpeed, double ySpeed, double wSpeed);
    void moveFieldCentric(double xSpeed, double ySpeed, double wSpeed, Translation2d centerOfRotation);
    ChassisSpeeds getSpeeds();
    void resetRobotAngle();
    void resetRobotAngle(double offset);
    double getRobotAngle();
    void driveByStates(SwerveModuleState
    [] states);
    SwerveDriveKinematics getKinematics();
    AHRSAngleGetterComponent getGyro();
}
