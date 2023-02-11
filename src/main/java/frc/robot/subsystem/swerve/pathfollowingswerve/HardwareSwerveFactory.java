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
import frc.robot.subsystem.swerve.SwerveConstants;

public class HardwareSwerveFactory {

    private static final double DRIVE_RATIO = 1/4.3329; //drive rotations per motor rotation
    private static final double ANGLE_RATIO = 1/12.34567901234; //angle rotations per motor rotation
    private static final double MAX_SPEED = SwerveConstants.MAX_LINEAR_SPEED; //max surface speed, meters per second

    private static final int DBRPORT = 2; //drive back right port
    private static final int ABRPORT = 3; //angle back right port
    private static final int DBLPORT = 12; //drive back left port
    private static final int ABLPORT = 11; //angle back left port
    private static final int DFRPORT = 4; //drive front right port
    private static final int AFRPORT = 5; //angle front right port
    private static final int DFLPORT = 7; //drive front left port
    private static final int AFLPORT = 6; //angle front left port


    private static final double WHEEL_DIAMETER = 0.0762; //Wheel diameter, in meters
    private static final double DRIVE_X_TRANSLATION = 0.2921; //lwft right translation of wheels
    private static final double DRIVE_Y_TRANSLATION = 0.2794; //front back translation of 
    static double DRIVE_Y_FRONT_TRANSLATION = 0.2032;
    static double DRIVE_Y_BACK_TRANSLATION = 0.4064;


    public static PathFollowingSwerve makeSwerve(){
        OdometricWheelModule fl = makeWheelModule(AFLPORT, DFLPORT, new Translation2d(DRIVE_Y_FRONT_TRANSLATION, DRIVE_X_TRANSLATION), true, true,true, .4, .75);
        OdometricWheelModule fr = makeWheelModule(AFRPORT, DFRPORT, new Translation2d(DRIVE_Y_FRONT_TRANSLATION, -DRIVE_X_TRANSLATION), true, true,false, .75, .75);
        OdometricWheelModule bl = makeWheelModule(ABLPORT, DBLPORT, new Translation2d(-DRIVE_Y_BACK_TRANSLATION, DRIVE_X_TRANSLATION), false, true,true, .9, .8);
        OdometricWheelModule br = makeWheelModule(ABRPORT, DBRPORT, new Translation2d(-DRIVE_Y_BACK_TRANSLATION, -DRIVE_X_TRANSLATION ), true, true,false, .6, .8);

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
                MAX_SPEED,
                WHEEL_DIAMETER,
                ANGLE_RATIO,
                DRIVE_RATIO);
    }
}