package frc.robot;

public class Constants {
    public static class SwerveConstants { // Swerve Constants
        public final static double MAX_LINEAR_SPEED = 5.8; // Change this!
        public final static double MAX_LINEAR_ACCELERATION = 4.8;
        public final static double MAX_ROTATIONAL_SPEED = Math.PI * 2 * 1.5;
        public final static double MAX_ROTATIONAL_ACCELERATION = Math.PI * 2 * 1.5;

        public static final double DRIVE_RATIO = 1/4.3329; //drive rotations per motor rotation
        public static final double ANGLE_RATIO = 1/12.34567901234; //angle rotations per motor rotation
        public static final double MAX_SPEED = SwerveConstants.MAX_LINEAR_SPEED; //max surface speed, meters per second

        public static final int DBRPORT = 2; //drive back right port
        public static final int ABRPORT = 3; //angle back right port
        public static final int DBLPORT = 12; //drive back left port
        public static final int ABLPORT = 11; //angle back left port
        public static final int DFRPORT = 4; //drive front right port
        public static final int AFRPORT = 5; //angle front right port
        public static final int DFLPORT = 7; //drive front left port
        public static final int AFLPORT = 6; //angle front left port


        public static final double WHEEL_DIAMETER = 0.0762; //Wheel diameter, in meters
        public static final double DRIVE_X_TRANSLATION = 0.2921; //lwft right translation of wheels
        public static final double DRIVE_Y_TRANSLATION = 0.2794; //front back translation of 
        public static double DRIVE_Y_FRONT_TRANSLATION = 0.2032;
        public static double DRIVE_Y_BACK_TRANSLATION = 0.4064;
    }

    public static class ArmConstants { // Arm Contstants
        public static final double ARM_DOWN_SENSOR_LIMIT = 1240; //TODO: fix all of these lol
        public static final double ARM_UP_SENSOR_LIMIT = 300;
        public static final double ARM_DOWN_ANGLE = 1.8;
        public static final double ARM_UP_ANGLE = 0.3;
    }

    public static class IntakeConstants { // Intake Constants
        public static final double intakeRunSpeed = -0.55; //TODO: fix
    }

    public static class VisionConstants { // Vision Constants
        /* Camera */
        public static final int width = 300;//TODO: fix
        public static final int height = 200;

        
        
    }

    public static class RobotConstants {//Robot Constants
        //TODO: add
    }
}