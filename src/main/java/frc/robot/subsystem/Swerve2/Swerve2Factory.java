package frc.robot.subsystem.Swerve2;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

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
            TalonFXComponent angleMotor = new TalonFXComponent(angleID);
            angleMotor.setSensorPhase(invertSensorPhase);
            angleMotor.setInverted(invertAngle);
            angleMotor.config_kP(0, anglekP);
            angleMotor.config_kF(0,anglekF);
            angleMotor.configMotionCruiseVelocity(10000);
            angleMotor.configMotionAcceleration(10000);
            angleMotor.configAllowableClosedloopError(0, 0);
            angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 26, 0.1));
            angleMotor.configClearPositionOnQuadIdx(true, 10);

            TalonFXComponent driveMotor = new TalonFXComponent(driveID);
            driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35, 36, 0.1));
            driveMotor.config_kP(0, .1);
            driveMotor.config_kI(0, 0);
            driveMotor.config_kD(0,0);
            driveMotor.config_kF(0, 0.047);
            driveMotor.config_IntegralZone(0, 0);
            driveMotor.setInverted(invertSpeed);
        return new Swerve2Module(angleMotor, driveMotor, translationFromCenter);
    }
}
