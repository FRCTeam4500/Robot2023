package frc.robot;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;

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
        public final static int BALANCE = 5;
        
        // Control stick map
        public final static int PLACE = 1;

        public final static int CUBE_INTAKE = 12;
        public final static int SIDEWAYS_CONE = 5;
        public final static int UPRIGHT_CONE = 6;

        public final static int READY_BOTTOM = 9;
        public final static int READY_MIDDLE = 7;
        public final static int READY_TOP = 8;
        public final static int SUBSTATION_PICKUP = 10;

    }

    public static class SwerveConstants { 
        public final static double MAX_LINEAR_SPEED = ((1276*9.42)/60)/12; // 1276 is rpm, 9.42 is wheel circumference (in.), final units are ft/s 
        public final static double MAX_LINEAR_ACCELERATION = 4; //Test
        public final static double MAX_ROTATIONAL_SPEED = MAX_LINEAR_SPEED / (4d/3); // 4/3 is (about) the radius from the center of the robot to the swerve drive wheels.
        public final static double MAX_ROTATIONAL_ACCELERATION = 4; // Linear Acceleration/radius

        public static final double DRIVE_RATIO = 1/5.; // drive rotations per motor rotation
        public static final double ANGLE_RATIO = 1/6.75; // angle rotations per motor rotation

        public static final int DBRPORT = 9; //drive back right port
        public static final int DBLPORT = 2; //drive back left port
        public static final int DFLPORT = 3; //drive front left port
        public static final int DFRPORT = 4; //drive front right port
        public static final int ABRPORT = 5; //angle back right port
        public static final int ABLPORT = 6; //angle back left port
        public static final int AFLPORT = 7; //angle front left port 
        public static final int AFRPORT = 8; //angle front right port


        public static final double WHEEL_DIAMETER = 0.0762; // in meters
        // distances from the gyro to the swerve module wheel.
        public static final double DRIVE_X_RIGHT_TRANSLATION = -0.2413; 
        public static final double DRIVE_X_LEFT_TRANSLATION = 0.2413; 
        public static final double DRIVE_Y_FRONT_TRANSLATION = 0.3175; 
        public static final double DRIVE_Y_BACK_TRANSLATION = -0.3175; 
    }
    

    public static class ArmConstants { // Arm Contstants
        /** The CAN ID of the arm tilt motor */
        public static final int TILT_MOTOR_ID = 10; 
        /** The motor type of the tilt motor */
        public static final MotorType TILT_MOTOR_TYPE = MotorType.kBrushless; 
        /** The CAN ID of the arm winch motor */
        public static final int WINCH_MOTOR_ID = 15;  

        /** The angle the arm must be at to pickup game pieces from the high substation <p> Units are whatever shuffleboard says*/
        public static final double ARM_HIGH_SUBSTATION_ANGLE = -3;
        /** The angle the arm must be at to place at the middle level <p> Units are whatever shuffleboard says*/
        public static final double ARM_PLACE_ANGLE = -6; 
        /** The angle the arm must be at to launch a cone onto the top node <p> Units are whatever shuffleboard says */
        public static final double ARM_LAUNCH_ANGLE = 1;
        /** The angle the arm must be at to pickup game pieces from the ground <p> Units are whatever shuffleboard says */
        public static final double ARM_GROUND_ANGLE = -43; 
        /** The angle the arm will go to while traveling <p> Units are whatever shuffleboard says */
        public static final double ARM_ZERO_ANGLE = -10; 

        /** The extension the arm must have to place a game piece on the top node <p> Also used for intaking game pieces off the high substation <p> Units are raw sensor units */
        public static final double ARM_PLACE_TOP = 10900.0; 
        /** The extension the arm must have to place a game piece on the middle node <p> Also used to place game pieces on the ground <p> Units are raw sensor units */
        public static final double ARM_PLACE_MID = 3229;
        /** The extension the arm must have to pickup a game piece from the ground <p> Units are raw sensor units*/
        public static final double ARM_PICKUP = 4324;
        /** The extension the arm will have while traveling <p> Units are raw sensor units */
        public static final double ARM_RETRACT = 0;
        /** Placing a bottom cone (as in we picked up a tilted cone) will require us to extend the arm this amount further <p> <Strong>This value needs to be updated</Strong> <p> Units are raw sensor units */
        public static final double ARM_BOT_CONE_ADDITION = 0; //TODO: Figure this out


    }

    public static class IntakeConstants { 
        /** The speed of the intake while it is intaking cones and placing cubes <p> Units are percentage of full power */
        public static final double INTAKE_CONE_SPEED = .8;
        /** The speed of the intake while it is intaking cubes and placing cones <p> Units are percentage of full power */
        public static final double INTAKE_CUBE_SPEED = -.9; 
        
        /** The CAN ID of the intake run motor */
        public static final int INTAKE_MOTOR_ID = 13;
        /** The motor type of the intake run motor */
        public static final MotorType INTAKE_MOTOR_TYPE = MotorType.kBrushless;
        /** The CAN ID of the intake angle motor */
        public static final int INTAKE_ANGLE_MOTOR_ID = 12;
        /** The motor type of the intake angle motor */
        public static final MotorType ANGLE_MOTOR_TYPE = MotorType.kBrushless;

        /** The angle the intake must be at to pickup game pieces from the ground <p> Units are whatever shuffleboard says */
        public static final double INTAKE_BOT_ANGLE = -7;
        /** The angle the intake must be at to pickup games pieces from the high substation <p> Units are whatever shuffleboard says */
        public static final double INTAKE_HIGH_SUBSTATION_ANGLE = -25;
        /** The angle the intake must be at to place a top cone (as in we picked up an upright cone) on a node <p> Units are whatever shuffleboard says */
        public static final double INTAKE_TOP_CONE_PLACE_ANGLE = -18.4;
        /** The angle the intake must be at to place a bottom cone (as in we picked up a sideways cone) on a node <p><strong> This value needs to be updated</strong><p> Units are whatever shuffleboard says */
        public static final double INTAKE_BOT_CONE_PLACE_ANGLE = -18.4; //TODO: Figure this out
        /** The angle the intake will go to while traveling <p> Units are whatever shuffleboard says */
        public static final double INTAKE_ZERO_ANGLE = -5.5; 
        /** The angle the intake must be at to launch a cone onto the top node <p> Units are whatever shuffleboard says */
        public static final double INTAKE_LAUNCHING_ANGLE = -15.5;
    }

    public static class VisionConstants { 
        
        public static final int width = 300;
        public static final int height = 200;
        /** IDK what this does, it the only variable in the old VisionConstants file, so I just moved it over <p> I think it says how close to the crosshair an object has to be to be counted as a target */
        public static final double MAXIMUM_ALLOWABLE_OFFSET = 0;

        public static final double VISION_HEIGHT = 0.0;
        public static final double VISION_ANGLE = 0.0;
        public static final double VISION_OFFSET = 0.0;

        public static final double TARGET_HEIGHT = 0.0;

        public static final double MAX_DEPTH_TO_TARGET = 0.1;
        public static final double MAX_OFFSET_TO_TARGET = 0.1;
    }

    public static class AutoConstants { //Everything in this class must be in meters
        
        /** The maximum velocity the robot will travel at during auto <p> Units are meters per second*/
        public static final double AUTO_MAX_SPEED = 2;
        /** The maximum acceleration the robot will travel at during auto <p> Units are meters per second*/
        public static final double AUTO_MAX_ACCEL = 2.0;

        public static final List<PathPlannerTrajectory> BlueBotRedTop2PieceTopAuto = PathPlanner.loadPathGroup(
        "BlueBotRedTop2PieceTop", AUTO_MAX_SPEED, AUTO_MAX_ACCEL);

        public static final List<PathPlannerTrajectory> BlueBotRedTop2PieceMidAuto = PathPlanner.loadPathGroup(
        "BlueBotRedTop2PieceMid", AUTO_MAX_SPEED, AUTO_MAX_ACCEL);

        public static final List<PathPlannerTrajectory> BlueBotRedTopPlaceAndDockAuto = PathPlanner.loadPathGroup(
        "BlueBotRedTopPlaceAndDock", AUTO_MAX_SPEED, AUTO_MAX_ACCEL);

        public static final List<PathPlannerTrajectory> BlueTopPlaceAndRunAuto = PathPlanner.loadPathGroup(
        "BlueTopPlaceAndRun", AUTO_MAX_SPEED, AUTO_MAX_ACCEL);

        public static final List<PathPlannerTrajectory> BlueTopRedBot2PieceTopAuto = PathPlanner.loadPathGroup(
        "BlueTopRedBot2PieceTop", AUTO_MAX_SPEED, AUTO_MAX_ACCEL);
        public static final List<PathPlannerTrajectory> BlueTopRedBot2PieceMidAuto = PathPlanner.loadPathGroup(
        "BlueTopRedBot2PieceMid", AUTO_MAX_SPEED, AUTO_MAX_ACCEL);

        public static final List<PathPlannerTrajectory> BlueTopRedBotPlaceAndDockAuto = PathPlanner.loadPathGroup(
        "BlueTopRedBotPlaceAndDock", AUTO_MAX_SPEED, AUTO_MAX_ACCEL);

        public static final List<PathPlannerTrajectory> MidPlaceAndDockAuto = PathPlanner.loadPathGroup(
        "MidPlaceAndDock", 1, 1);

        public static final List<PathPlannerTrajectory> PlaceAndMoveAuto = PathPlanner.loadPathGroup(
        "PlaceAndMove", AUTO_MAX_SPEED, AUTO_MAX_ACCEL);

        public static final List<PathPlannerTrajectory> RedTopPlaceAndRunAuto= PathPlanner.loadPathGroup(
        "RedTopPlaceAndRun", AUTO_MAX_SPEED, AUTO_MAX_ACCEL); 
        // Add Auto Paths here, like the above
    }

    public static class RobotConstants {
        /** A hash map containing the generic commands the robot will use <p> These commands can be accessed by putting the cooresponding string key into the .get() method
         * <p> Example: {@code commandMap.get("zero");} 
         */
        public static final HashMap<String, Command> commandMap = new HashMap<>();
    }
}