package frc.robot.subsystem.Swerve2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants.SwerveConstants;
import frc.robot.component.hardware.AHRSAngleGetterComponent;
import frc.robot.component.hardware.TalonFXComponent;

public class Swerve2Factory {
    private static AHRSAngleGetterComponent gyro = new AHRSAngleGetterComponent(I2C.Port.kMXP);

    private static Swerve2Module frontLeftModule;
    private static Swerve2Module frontRightModule;
    private static Swerve2Module backLeftModule;
    private static Swerve2Module backRightModule;

    private static TalonFXComponent angleMotor;
    private static TalonFXComponent driveMotor;

    public static Swerve2 makeSwerve2() {
        frontLeftModule = makeSwerve2Module(
            SwerveConstants.AFLPORT, SwerveConstants.DFLPORT, 
            new Translation2d(SwerveConstants.DRIVE_Y_FRONT_TRANSLATION, SwerveConstants.DRIVE_X_LEFT_TRANSLATION), 
            false, false,true, .3, .75);

        frontRightModule = makeSwerve2Module(SwerveConstants.AFRPORT, SwerveConstants.DFRPORT, 
            new Translation2d(SwerveConstants.DRIVE_Y_FRONT_TRANSLATION, SwerveConstants.DRIVE_X_RIGHT_TRANSLATION), 
            false, false,false, .3, .75);

        backLeftModule = makeSwerve2Module(SwerveConstants.ABLPORT, SwerveConstants.DBLPORT, 
            new Translation2d(SwerveConstants.DRIVE_Y_BACK_TRANSLATION, SwerveConstants.DRIVE_X_LEFT_TRANSLATION), 
            false, false,true, .3, .8);

        backRightModule = makeSwerve2Module(SwerveConstants.ABRPORT, SwerveConstants.DBRPORT, 
            new Translation2d(SwerveConstants.DRIVE_Y_BACK_TRANSLATION, SwerveConstants.DRIVE_X_RIGHT_TRANSLATION ), 
            false, false,false, .3, .8);

        return new Swerve2(gyro, frontLeftModule, frontRightModule, backLeftModule, backRightModule);
    }

    public static Swerve2Module makeSwerve2Module(
        int angleID, int driveID, 
        Translation2d translationFromCenter, 
        boolean invertSensorPhase, boolean invertAngle, boolean invertSpeed,
        double anglekP, double anglekF) {
            
        return new Swerve2Module();
    }
}
