package frc.robot.subsystem.Swerve2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.component.hardware.AHRSAngleGetterComponent;

public class Swerve2 extends SubsystemBase{
    private SwerveDriveKinematics kinematics;
    private Swerve2Module[] modules;
    private SwerveModulePosition[] positions;
    private AHRSAngleGetterComponent gyro;
    private Translation2d[] translations;
    private SwerveDriveOdometry odometry;

    public Swerve2(AHRSAngleGetterComponent gyro, Swerve2Module... swerve2Modules) {
        modules = swerve2Modules; 
        this.gyro = gyro;
        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Math.toRadians(gyro.getAngle())), getModulePositions());
    }

    @Override
    public void periodic() {
        
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
}
