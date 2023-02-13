package frc.robot;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;;

public class Constants {
    public static class JoystickConstants {
        // Drive stick map
        public final static int LOCK_SWERVE_ROTATION = 1;
        public final static int SWITCH_DRIVE_MODE_ROBOT_CENTRIC = 4;
        public final static int ALIGN_SWERVE_TO_ANGLE = 8;
        public final static int ALIGN_SWERVE_REVERSE = 7;
        public final static int RESET_GYRO = 10;
        public final static int LIMIT_SWERVE_SPEED = 2;
        public final static int NO_FORWARD = 9;
        public final static int BALANCE = 11;

        // Control stick map
        public final static int CONE_INTAKE = 11; // Also places cubes
        public final static int CUBE_INTAKE = 12; // Also places cones 
        public final static int PLACE = 1;
        public final static int READY_GROUND = 10;
        public final static int READY_BOTTOM = 0;
        public final static int READY_MIDDLE = 0;
        public final static int READY_TOP = 0;
        public final static int RETRACT = 0;
        public final static int UPRIGHT_CONE = 0;
        public final static int SIDEWAYS_CONE = 0;

    }

    public static class SwerveConstants { // Swerve Constants
        public final static double MAX_LINEAR_SPEED = 5.088432798984; // Change this!
        public final static double MAX_LINEAR_ACCELERATION = ((1276*9.42)/60)/12; // 1276 is rpm, 9.42 is wheel circumference (in.)
        public final static double MAX_ROTATIONAL_SPEED = 0.0;
        public final static double MAX_ROTATIONAL_ACCELERATION = 0.0;

        public static final double DRIVE_RATIO = 1/5; // drive rotations per motor rotation
        public static final double ANGLE_RATIO = 1/6.75; // angle rotations per motor rotation

        public static final int DBRPORT = 2; //drive back right port
        public static final int ABRPORT = 3; //angle back right port
        public static final int DBLPORT = 12; //drive back left port
        public static final int ABLPORT = 11; //angle back left port
        public static final int DFRPORT = 4; //drive front right port
        public static final int AFRPORT = 5; //angle front right port
        public static final int DFLPORT = 7; //drive front left port
        public static final int AFLPORT = 6; //angle front left port TODO: Change All Motor Map Values


        public static final double WHEEL_DIAMETER = 0.0762; // in meters
        public static final double DRIVE_X_TRANSLATION = 0.2921; // left right translation of wheels, 11.5 inches
        public static final double DRIVE_Y_TRANSLATION = 0.2794; // front back translation of wheels
        public static double DRIVE_Y_FRONT_TRANSLATION = 0.2032; // 8 inches from gyro?
        public static double DRIVE_Y_BACK_TRANSLATION = 0.4064; // 16 inches
    }

    public static class ArmConstants { // Arm Contstants
        /* Init */
        public static final int TILT_MOTOR_ID = 0; //TODO: fix
        public static final MotorType TILT_MOTOR_TYPE = MotorType.kBrushless; 
        public static final int WINCH_MOTOR_ID = 0; //TODO: fix
        public static final MotorType WINCH_MOTOR_TYPE = MotorType.kBrushless; 

        /* Force Limit */
        public static final double ARM_DOWN_SENSOR_LIMIT = 1240; //TODO: fix all of these lol
        public static final double ARM_UP_SENSOR_LIMIT = 300;

        public static final double ARM_PLACE_ANGLE = 0; // ARM_PLACE_ANGLE is the angle for all heights TODO: change value later

        public static final double ARM_PICKUP_ANGLE = 0; // ARM_PICKUP_ANGLE is the angle to pickup from ground TODO: change value later

        public static final double ARM_ZERO_ANGLE = 0; // TODO: change value later

        public static final double ARM_PLACE_TOP = 0; // TODO: change value later

        public static final double ARM_PLACE_MID = 0; // TODO: change value later

        public static final double ARM_PLACE_BOT = 0; // TODO: change value later

        public static final double ARM_RETRACT = 0; // TODO: change value later

        public static final double ARM_BOTTOM_CONE_INTAKE_ADDITION = 0; // TODO: change value later

    }

    public static class IntakeConstants { // Intake Constants
        public static final double INTAKE_CUBE_SPEED = 0.55; //TODO: fix
        public static final double INTAKE_CONE_SPEED = -0.55;

        public static final int INTAKE_MOTOR_ID = 0; //TODO: fix
        public static final MotorType INTAKE_MOTOR_TYPE = MotorType.kBrushless;
        public static final int INTAKE_ANGLE_MOTOR_ID = 0; //TODO: fix
        public static final MotorType ANGLE_MOTOR_TYPE = MotorType.kBrushless;

        public static final double INTAKE_BOTTOM_ANGLE = 0; // Angle that the Intake should be at the bottom when picking up TODO: Fix value

        public static final double INTAKE_TRAY_ANGLE = 0; // INTAKE_TRAY_ANGLE is the angle for placing TODO: Fix value | also will place top cone
        public static final double INTAKE_RETRACTED_ANGLE = 0; // TRUE retracted
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