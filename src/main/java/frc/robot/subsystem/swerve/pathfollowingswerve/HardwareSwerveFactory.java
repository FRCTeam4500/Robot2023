package frc.robot.subsystem.swerve.pathfollowingswerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.component.hardware.AHRSAngleGetterComponent;
import frc.robot.component.hardware.TalonFXComponent;
import frc.robot.component.hardware.TalonSRXComponent;
import frc.robot.Constants.SwerveConstants;

public class HardwareSwerveFactory {

    public static PathFollowingSwerve makeSwerve(){
        OdometricWheelModule fl = makeWheelModule(SwerveConstants.AFLPORT, SwerveConstants.DFLPORT, new Translation2d(SwerveConstants.DRIVE_Y_FRONT_TRANSLATION, SwerveConstants.DRIVE_X_LEFT_TRANSLATION), true, true,true, .4, .75);
        OdometricWheelModule fr = makeWheelModule(SwerveConstants.AFRPORT, SwerveConstants.DFRPORT, new Translation2d(SwerveConstants.DRIVE_Y_FRONT_TRANSLATION, -SwerveConstants.DRIVE_X_RIGHT_TRANSLATION), true, true,false, .75, .75);
        OdometricWheelModule bl = makeWheelModule(SwerveConstants.ABLPORT, SwerveConstants.DBLPORT, new Translation2d(-SwerveConstants.DRIVE_Y_BACK_TRANSLATION, SwerveConstants.DRIVE_X_LEFT_TRANSLATION), false, true,true, .9, .8);
        OdometricWheelModule br = makeWheelModule(SwerveConstants.ABRPORT, SwerveConstants.DBRPORT, new Translation2d(-SwerveConstants.DRIVE_Y_BACK_TRANSLATION, -SwerveConstants.DRIVE_X_RIGHT_TRANSLATION ), true, true,false, 1, .8);

        return new OdometricSwerve(
                new AHRSAngleGetterComponent(I2C.Port.kMXP),
                fl,
                fr,
                bl,
                br
        );
    }

    /**
     * 
     * @param angleId
     * @param driveId
     * @param translationFromSwerveCenter The swerve center is at the gyro
     * @param invertSensorPhase
     * @param invertAngle
     * @param invertSpeed
     * @param anglekP
     * @param anglekF
     * @return
     */
    public static OdometricWheelModule makeWheelModule(int angleId, int driveId,Translation2d translationFromSwerveCenter, boolean invertSensorPhase, boolean invertAngle, boolean invertSpeed,
    double anglekP, double anglekF){
        TalonFXComponent angleMotor = new TalonFXComponent(angleId);
        angleMotor.setSensorPhase(invertSensorPhase);
        angleMotor.setInverted(invertAngle);
        angleMotor.config_kP(0, anglekP);
        angleMotor.config_kF(0,anglekF);
        angleMotor.configMotionCruiseVelocity(10000);
        angleMotor.configMotionAcceleration(10000);
        angleMotor.configAllowableClosedloopError(0, 0);
        angleMotor.configClearPositionOnQuadIdx(true, 10);

        TalonFXComponent driveMotor = new TalonFXComponent(driveId);
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 41, 0.1));
        driveMotor.config_kP(0, .1);
        driveMotor.config_kI(0, 0);
        driveMotor.config_kD(0,0);
        driveMotor.config_kF(0, 0.047);
        driveMotor.config_IntegralZone(0, 0);
        driveMotor.setInverted(invertSpeed);
        return new OdometricWheelModule(
                angleMotor,
                driveMotor,
                translationFromSwerveCenter,
                SwerveConstants.MAX_LINEAR_SPEED,
                SwerveConstants.WHEEL_DIAMETER,
                SwerveConstants.ANGLE_RATIO,
                SwerveConstants.DRIVE_RATIO);
    }
}