package frc.robot.subsystem.swerve.pathfollowingswerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.component.hardware.AHRSAngleGetterComponent;
import frc.robot.component.hardware.TalonFXComponent;
import frc.robot.subsystem.swerve.SwerveConstants;

public class HardwareSwerveFactory {

    private static final double DRIVE_RATIO = 1/5.; //drive rotations per motor rotation
    private static final double ANGLE_RATIO = 1/6.75; //angle rotations per motor rotation
    private static final double MAX_SPEED = SwerveConstants.MAX_LINEAR_SPEED; //max surface speed, meters per second

    private static final int DBRPORT = 9; //drive back right port
    private static final int ABRPORT = 5; //angle back right port
    private static final int DBLPORT = 2; //drive back left port
    private static final int ABLPORT = 6; //angle back left port
    private static final int DFRPORT = 4; //drive front right port
    private static final int AFRPORT = 8; //angle front right port
    private static final int DFLPORT = 3; //drive front left port
    private static final int AFLPORT = 7; //angle front left port


    public static final double WHEEL_DIAMETER = 0.0762; // in meters
    public static final double DRIVE_X_RIGHT_TRANSLATION = -0.2413; // distance from gyro to swerve _>NOTE: Right is negative!!!!
    public static final double DRIVE_X_LEFT_TRANSLATION = 0.2413; // front back translation of wheels
    public static final double DRIVE_Y_FRONT_TRANSLATION = 0.3175; // 8 inches from gyro?
    public static final double DRIVE_Y_BACK_TRANSLATION = -0.3175; // 16 inches

    public static PathFollowingSwerve makeSwerve(){
        OdometricWheelModule fl = makeWheelModule(AFLPORT, DFLPORT, new Translation2d(DRIVE_Y_FRONT_TRANSLATION, DRIVE_X_LEFT_TRANSLATION), false, false,true, .3, .75);
        OdometricWheelModule fr = makeWheelModule(AFRPORT, DFRPORT, new Translation2d(DRIVE_Y_FRONT_TRANSLATION, DRIVE_X_RIGHT_TRANSLATION), false, false,false, .3, .75);
        OdometricWheelModule bl = makeWheelModule(ABLPORT, DBLPORT, new Translation2d(DRIVE_Y_BACK_TRANSLATION, DRIVE_X_LEFT_TRANSLATION), false, false,true, .3, .8);
        OdometricWheelModule br = makeWheelModule(ABRPORT, DBRPORT, new Translation2d(DRIVE_Y_BACK_TRANSLATION, DRIVE_X_RIGHT_TRANSLATION ), false, false,false, .3, .8); // Inverted speeds for back drive motors due to opposite splin, calibrated for back two(gear in), front two(gear out). Made it so that it's both gear out :)

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