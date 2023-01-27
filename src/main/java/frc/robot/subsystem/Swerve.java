package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.component.hardware.TalonFXComponent;
import static frc.robot.utility.ExtendedMath.getShortestRadianToTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.component.AngleComponent;
import frc.robot.component.AngularVelocityComponent;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.component.GyroComponent;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.component.hardware.AHRSAngleGetterComponent;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants.SwerveConstants;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.utility.ControllerInfo;
import edu.wpi.first.math.controller.PIDController;

/**
 * Swerve Drive {Make sure to change Constants and add docs}
 */
public class Swerve {

    /* Constants for Factory, Change in Constants File */
    private static final double DRIVE_RATIO = SwerveConstants.DRIVE_RATIO; //drive rotations per motor rotation
    private static final double ANGLE_RATIO = SwerveConstants.ANGLE_RATIO; //angle rotations per motor rotation
    private static final double MAX_SPEED = SwerveConstants.MAX_LINEAR_SPEED; //max surface speed, meters per second

    private static final int DBRPORT = SwerveConstants.DBRPORT; //drive back right port
    private static final int ABRPORT = SwerveConstants.ABRPORT; //angle back right port
    private static final int DBLPORT = SwerveConstants.DBLPORT; //drive back left port
    private static final int ABLPORT = SwerveConstants.ABLPORT; //angle back left port
    private static final int DFRPORT = SwerveConstants.DFRPORT; //drive front right port
    private static final int AFRPORT = SwerveConstants.AFRPORT; //angle front right port
    private static final int DFLPORT = SwerveConstants.DFLPORT; //drive front left port
    private static final int AFLPORT = SwerveConstants.AFLPORT; //angle front left port


    private static final double WHEEL_DIAMETER = SwerveConstants.WHEEL_DIAMETER; //Wheel diameter, in meters
    private static final double DRIVE_X_TRANSLATION = SwerveConstants.DRIVE_X_TRANSLATION; //lwft right translation of wheels
    private static double DRIVE_Y_FRONT_TRANSLATION = SwerveConstants.DRIVE_Y_FRONT_TRANSLATION;
    private static double DRIVE_Y_BACK_TRANSLATION = SwerveConstants.DRIVE_Y_BACK_TRANSLATION;


    /* Wheel Modules */
    /* Application to Kinematic */
    public static class KinematicWheelModule extends SubsystemBase {
        protected AngleComponent angleComponent;
        protected double angleRotsPerMotorRots;
        protected double driveRotsPerMotorRots;
        protected AngularVelocityComponent angularVelocityComponent;
        protected Translation2d translationFromSwerveCenter;
        protected double maxSurfaceSpeed;
        protected double wheelDiameter;
        public KinematicWheelModule(AngleComponent angleComponent, AngularVelocityComponent angularVelocityComponent, Translation2d translationFromSwerveCenter, double maxSurfaceSpeed, double wheelDiameter, double angleRotsPerMotorRots, double driveRotsPerMotorRots) {
            this.angleComponent = angleComponent;
            this.angularVelocityComponent = angularVelocityComponent;
            this.translationFromSwerveCenter = translationFromSwerveCenter;
            this.maxSurfaceSpeed = maxSurfaceSpeed;
            this.wheelDiameter = wheelDiameter;
            this.angleRotsPerMotorRots = angleRotsPerMotorRots;
            this.driveRotsPerMotorRots = driveRotsPerMotorRots;
        }
        public void drive(SwerveModuleState state) {
            angleComponent.setAngle(state.angle.getRadians() / angleRotsPerMotorRots);
            angularVelocityComponent.setAngularVelocity(state.speedMetersPerSecond / (wheelDiameter * Math.PI) * 2 * Math.PI / driveRotsPerMotorRots);
        }
        public Translation2d getTranslationFromSwerveCenter() {
            return translationFromSwerveCenter;
        }

        public double getMaxSurfaceSpeed() {
            return maxSurfaceSpeed;
        }
        @Override
        public void periodic() {
            // This method will be called once per scheduler run
        }
        public SwerveModuleState getState() {
            return new SwerveModuleState(angularVelocityComponent.getAngularVelocity(), new Rotation2d(angleComponent.getAngle()));
        }
    }
    
    /* Application to Odemetric */
    public static class OdometricWheelModule extends KinematicWheelModule {
        protected TalonFXComponent driveMotor;
        protected TalonFXComponent angleMotor;
        protected boolean wheelWrapEnabled = true;
        protected boolean wheelInversionEnabled = true;

        public OdometricWheelModule(TalonFXComponent angleMotor, TalonFXComponent driveMotor, Translation2d translationFromSwerveCenter,
                                    double maxSurfaceSpeed, double wheelDiameter,
                                    double angleRotsPerMotorRots, double driveRotsPerMotorRots) {
            super(angleMotor, driveMotor, translationFromSwerveCenter, maxSurfaceSpeed, wheelDiameter, angleRotsPerMotorRots, driveRotsPerMotorRots);
            this.driveMotor = driveMotor;
            this.angleMotor = angleMotor;
        }


        public SwerveModuleState getState() {
            return new SwerveModuleState(driveMotor.getAngularVelocity() * driveRotsPerMotorRots / 2
                    / Math.PI * Math.PI * wheelDiameter,
                    new Rotation2d(angleMotor.getAngle() * angleRotsPerMotorRots));
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition((driveMotor.getSelectedSensorPosition()/2048/5.3625)*(2*Math.PI*0.0381), new Rotation2d(angleMotor.getAngle() * angleRotsPerMotorRots));
        }

        @Override
        public void drive(SwerveModuleState state) {
            if (wheelWrapEnabled) {
                double currentAngle = angleMotor.getAngle() *angleRotsPerMotorRots;

                // Get the closest angle equivalent to the target angle. Otherwise the wheel
                // module is going
                // to spin a lot since it will be at
                // something like 3 pi and the target will be 0 pi; the motor will move 3 pi
                // instead of just
                // 1 pi without this bit
                double shortestRadianToTarget = getShortestRadianToTarget(currentAngle, state.angle.getRadians());
                double targetAngle = shortestRadianToTarget + currentAngle;

                if (wheelInversionEnabled) {
                    // Sometimes it will be easier to reverse the speed instead of rotating the
                    // whole module by
                    // pi radians.
                    // These lines determine whether this is needed.
                    double oppositeAngle = targetAngle + Math.PI;
                    double shortestDistanceToOppositeAngle = getShortestRadianToTarget(currentAngle, oppositeAngle);
                    double finalAngle;
                    double finalSpeed;
                    if (Math.abs(shortestDistanceToOppositeAngle) < Math.abs(shortestRadianToTarget)) {
                        finalAngle = currentAngle + shortestDistanceToOppositeAngle;
                        finalSpeed = -state.speedMetersPerSecond;
                    } else {
                        finalAngle = currentAngle + shortestRadianToTarget;
                        finalSpeed = state.speedMetersPerSecond;
                    }
                    state.angle = new Rotation2d(finalAngle);
                    state.speedMetersPerSecond = finalSpeed;
                }else{
                    state.angle = new Rotation2d(targetAngle);
                }
            }
            super.drive(state);
        }
        public void coast() {
            driveMotor.setOutput(0);
        }
    }

    public static class KinematicSwerve extends SubsystemBase {

        protected SwerveDriveKinematics kinematics;
        protected KinematicWheelModule[] wheelModules;
        protected double lowestMaximumWheelSpeed;
        protected double currentGyroZero = 0.0;
        protected GyroComponent gyro;
        private int resetCounter;
        private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
        /**
         * Creates a new KinematicSwerve.
         */
        public KinematicSwerve(GyroComponent gyro, KinematicWheelModule... wheelModules) {
            this.wheelModules = wheelModules;
            this.gyro = gyro;
    
            kinematics = new SwerveDriveKinematics(getTranslations(wheelModules));
    
            lowestMaximumWheelSpeed = getLowestMaximumWheelModuleSpeeds(wheelModules);
    
        }
    
        @Override
        public void periodic() {
            if (resetCounter > 500){
                resetWheels();
            }
            else{
                resetCounter++;
            }
            // This method will be called once per scheduler run
        }
        /**
         * A wrapper for {@link #moveRobotCentric(ChassisSpeeds)}.
         * @param xSpeed Forward speed in meters/second
         * @param ySpeed Leftward speed in meters/second
         * @param wSpeed Counterclockwise rotational speed in radians/second
         */
        public void moveRobotCentric(double xSpeed, double ySpeed, double wSpeed){
            resetCounter = 0;
            var chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, wSpeed);
            moveRobotCentric(chassisSpeeds);
        }
        /**
         * Move the swerve drive relative to itself using the desired {@link ChassisSpeeds}.
         * @param chassisSpeeds the target chassis speeds for the swerve drive.
         * @see #moveRobotCentric(double, double, double)
         */
        public void moveRobotCentric(ChassisSpeeds chassisSpeeds){
            moveRobotCentric(chassisSpeeds, new Translation2d());
        }
        public void moveRobotCentric(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){
            currentSpeeds = chassisSpeeds;
            var states = kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
            driveByStates(states);
    
        }
    
        /**
         * Drives the robots using the raw module states rather than by chassis speeds
         * For use with autonomous commands
         * @param states
         */
        public void driveByStates(SwerveModuleState[] states) {
            SwerveDriveKinematics.desaturateWheelSpeeds(states, lowestMaximumWheelSpeed);
    
            for (int i = 0; i < wheelModules.length; i++) {
                wheelModules[i].drive(states[i]);
            }
        }
        /**
         * A wrapper for {@link #moveAngleCentric(double, double, double, Rotation2d)}
         */
        public void moveAngleCentric(double xSpeed, double ySpeed, double wSpeed, double robotAngle){
            moveAngleCentric(xSpeed, ySpeed, wSpeed, new Rotation2d(robotAngle));
        }
        /**
         * Move the swerve drive relative to an angle. This angle is usually the gyroscope reading. When the angle is zero, the swerve drive is assumed to face the positive X direction, and positive Y is directly to the left of the swerve drive.
         * @param xSpeed the forward speed to move when angle is zero in meters/second
         * @param ySpeed the leftward speed to move when angle is zero in meters/second
         * @param wSpeed the counterclockward speed to rotate in radians/second
         * @param robotAngle the angle of the robot relative to the described coordinate system
         */
        public void moveAngleCentric(double xSpeed, double ySpeed, double wSpeed, Rotation2d robotAngle){
            moveAngleCentric(xSpeed, ySpeed, wSpeed, robotAngle, new Translation2d());
        }
        public void moveAngleCentric(double xSpeed, double ySpeed, double wSpeed, Rotation2d robotAngle, Translation2d centerOfRotation) {
            var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, wSpeed, robotAngle);
            moveRobotCentric(chassisSpeeds, centerOfRotation);
        }
        public void moveFieldCentric(double xSpeed, double ySpeed, double wSpeed){
            moveAngleCentric(xSpeed, ySpeed, wSpeed, gyro.getAngle() - currentGyroZero);
        }
        public void moveFieldCentric(ChassisSpeeds speeds){
            moveFieldCentric(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        }
        public void moveFieldCentric(double xSpeed, double ySpeed, double wSpeed, Translation2d centerOfRotation){
            moveAngleCentric(xSpeed, ySpeed, wSpeed, new Rotation2d(gyro.getAngle() - currentGyroZero),centerOfRotation);
        }
    
        private Translation2d[] getTranslations(KinematicWheelModule[] wheelModules){
            var translations = new Translation2d[wheelModules.length];
            for(int i = 0;i < wheelModules.length;i++){
                translations[i] = wheelModules[i].getTranslationFromSwerveCenter();
            }
            return translations;
        }
        private double getLowestMaximumWheelModuleSpeeds(KinematicWheelModule[] wheelModules){
            var lowestMaxSpeed = Double.MAX_VALUE;
            for(var module : wheelModules){
                lowestMaxSpeed = Math.min(module.getMaxSurfaceSpeed(), lowestMaxSpeed);
            }
            return lowestMaxSpeed;
        }
        
    
        public double getRobotAngle(){
            return gyro.getAngle() - currentGyroZero;
        }
    
        public void resetRobotAngle(){
            currentGyroZero = gyro.getAngle();
        }
        /**
         * Resets robot angle with an offset
         * @param angle
         */
        public void resetRobotAngle(double angle){
            currentGyroZero = gyro.getAngle() - angle;
        }
    
        public void resetWheels(){
            for (KinematicWheelModule wheel : wheelModules){
                wheel.angleComponent.setAngle(0);
            }
        }
    
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty("Gyro Angle", gyro::getAngle, null);
            builder.addDoubleProperty("Current X", () -> currentSpeeds.vxMetersPerSecond, null);
            builder.addDoubleProperty("Current Y", () -> currentSpeeds.vyMetersPerSecond, null);
            builder.addDoubleProperty("Current Z", () -> currentSpeeds.omegaRadiansPerSecond, null);
        }
    
        public ChassisSpeeds getSpeeds(){
            SwerveModuleState[] states = {
                wheelModules[0].getState(),
                wheelModules[1].getState(),
                wheelModules[2].getState(),
                wheelModules[3].getState()
            };
            return kinematics.toChassisSpeeds(states);
        }
    
        public SwerveDriveKinematics getKinematics(){
            return kinematics;
        }
    }

    public static class OdometricSwerve extends KinematicSwerve {

        SwerveDriveOdometry odometry;
        OdometricWheelModule[] odometricWheelModules;
        private Translation2d lastTranslation;
        private double lastTime;
        public OdometricSwerve(GyroComponent gyro, OdometricWheelModule... wheelModules) {
            super(gyro, wheelModules);
            odometricWheelModules = wheelModules;
            odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getAngle()), getSwerveModulePositions());
        }
        private void updateOdometry(){
            lastTime = Timer.getFPGATimestamp();
            lastTranslation = getCurrentPose().getTranslation();
            odometry.update(new Rotation2d(gyro.getAngle()), getSwerveModulePositions());
        }

        /* Currently Out of Use, using getSwerveModulePositions() instead :) */
        private SwerveModuleState[] getSwerveModuleStates(){
            var states = new SwerveModuleState[wheelModules.length];
            for(int i = 0;i<states.length;i++){
                states[i] = odometricWheelModules[i].getState();
            }
            return states;
        }

        private SwerveModulePosition[] getSwerveModulePositions() {
            var positions = new SwerveModulePosition[wheelModules.length];
            for(int i = 0;i<positions.length;i++){
                positions[i] = odometricWheelModules[i].getPosition();
            }
            return positions;
        }
    
        public Pose2d getCurrentPose(){
            return odometry.getPoseMeters();
        }
        public Translation2d getCurrentVelocity(){
            return getCurrentPose().getTranslation().minus(lastTranslation).div(Timer.getFPGATimestamp() - lastTime);
        }
        public void resetPose(){
            resetRobotAngle();
            odometry.resetPosition(new Rotation2d(gyro.getAngle()), getSwerveModulePositions(), new Pose2d(getCurrentPose().getTranslation(),new Rotation2d()));
        }
        public void resetPose(Translation2d translation){
            resetRobotAngle();
            odometry.resetPosition(new Rotation2d(gyro.getAngle()), getSwerveModulePositions(), new Pose2d(translation, new Rotation2d()));
        }
        public void resetPose(Pose2d pose) {
            resetRobotAngle(pose.getRotation().getRadians());
            odometry.resetPosition(new Rotation2d(gyro.getAngle()), getSwerveModulePositions(), pose);
        }
        public void periodic() {
            updateOdometry();
        }
        public void enableWheelWrap(boolean enable){
            for(var module : odometricWheelModules) {
                module.wheelWrapEnabled = enable;
            }
        }
        public void enableWheelInversion(boolean enable){
            for(var module : odometricWheelModules) {
                module.wheelInversionEnabled = enable;
            }
        }
        public void coast() {
            for(var module : odometricWheelModules) {
                module.coast();
            }
        }
    
        @Override
        public void initSendable(SendableBuilder builder) {
            super.initSendable(builder);
            builder.addDoubleProperty("Odometric rotation", () -> getCurrentPose().getRotation().getDegrees(), null);
            builder.addDoubleProperty("Odometric X", () -> getCurrentPose().getX(), null);
            builder.addDoubleProperty("Odometric Y", () -> getCurrentPose().getY(), null);
        }
    
    }
    public static OdometricSwerve makeSwerve() {
        OdometricWheelModule fl = makeWheelModule(AFLPORT, DFLPORT, new Translation2d(DRIVE_Y_FRONT_TRANSLATION, DRIVE_X_TRANSLATION), true, true,true, .4, .75);
        OdometricWheelModule fr = makeWheelModule(AFRPORT, DFRPORT, new Translation2d(DRIVE_Y_FRONT_TRANSLATION, -DRIVE_X_TRANSLATION), true, true,false, .75, .75);
        OdometricWheelModule bl = makeWheelModule(ABLPORT, DBLPORT, new Translation2d(-DRIVE_Y_BACK_TRANSLATION, DRIVE_X_TRANSLATION), false, true,true, .9, .8);
        OdometricWheelModule br = makeWheelModule(ABRPORT, DBRPORT, new Translation2d(-DRIVE_Y_BACK_TRANSLATION, -DRIVE_X_TRANSLATION ), true, true,false, 1, .8);

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

    public static class SwerveCommand extends CommandBase {
        private OdometricSwerve swerve;
        private Joystick joystick;
        private ControllerInfo info;
    
        private PIDController angleAdjustmentController;
        public ControlMode controlMode;
    
        public boolean lockRotation = false;
        public boolean limitSpeed = false;
        public boolean noForward = false;
        public double targetAngle = 0;
    
        private double limitedSpeed = .75;
    
        public SwerveCommand(OdometricSwerve swerve, Joystick joystick, ControllerInfo controllerInfo) {
            this.swerve = swerve;
            this.joystick = joystick;
            info = controllerInfo;
            controlMode = ControlMode.FieldCentric; // default control mode is field-centric
            angleAdjustmentController = new PIDController(1,0,0);
            angleAdjustmentController.enableContinuousInput(-Math.PI, Math.PI);
            addRequirements(swerve);
        }
    
        @Override
        public void execute(){
            double xSpeed = -withDeadzone(joystick.getX(), info.xDeadzone) * info.xSensitivity;
            double ySpeed = -withDeadzone(joystick.getY(), info.yDeadzone) * info.ySensitivity;
            double zSpeed = -withDeadzone(joystick.getZ(), info.zDeadzone) * info.zSensitivity;
            if (limitSpeed){
                xSpeed = ceiling(xSpeed, limitedSpeed);
                ySpeed = ceiling(ySpeed, limitedSpeed);
                zSpeed = ceiling(zSpeed, limitedSpeed);
            }
            if (lockRotation) {
                zSpeed = 0;
            }
            if (noForward) {
                ySpeed = 0;
                zSpeed = 0;
                xSpeed = ceiling(xSpeed, limitedSpeed);
            }        
            switch (controlMode){
                case FieldCentric:
                    moveFieldCentric(xSpeed, ySpeed, zSpeed);
                    break;
                case RobotCentric:
                    moveRobotCentric(xSpeed,ySpeed,zSpeed);
                    break;
                case AlignToAngle:
                    moveAlign(xSpeed,ySpeed,zSpeed);
                    break;
            }
        }
    
        private void moveFieldCentric(double x, double y, double w){
            swerve.moveFieldCentric(y,x,w);
        }
        private void moveRobotCentric(double x, double y, double w){
            swerve.moveRobotCentric(y,x,w);
        }
        private void moveAlign(double r, double t, double w) {
            double wSpeed = 4 * angleAdjustmentController.calculate(swerve.getRobotAngle(), targetAngle);
            moveFieldCentric(r, t, wSpeed);
        }
        //deadzones the input
        private double withDeadzone(double value, double deadzone){
            if(Math.abs(value) < deadzone)
                return 0;
            else
                return value;
        }
    
        public enum ControlMode{
            FieldCentric,
            RobotCentric,
            AlignToAngle
        }
    
        private double ceiling(double value, double maximum){
            if (Math.abs(value) > maximum){
                return maximum * Math.signum(value);
            }
            return value;
        }
    
        public void initSendable(SendableBuilder builder){
            builder.addStringProperty("Drive Mode", () -> {
                switch (controlMode) {
                    case FieldCentric:
                        return "Field Centric";
                
                    case RobotCentric:
                        return "Robot Centric";
    
                    case AlignToAngle:
                        return "Align To Angle";
                }
                return "";
            }, null);
            builder.addDoubleProperty("controller x", joystick::getX, null);
            builder.addDoubleProperty("controller y", joystick::getY, null);
            builder.addDoubleProperty("controller z", joystick::getZ, null);
            builder.addBooleanProperty("Limit Speed", () -> {return limitSpeed;}, null);
        }
    }
}
