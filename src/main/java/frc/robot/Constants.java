package frc.robot;

public static class Constants {
    public static class Swerve { // Swerve Constants
        public final static double MAX_LINEAR_SPEED = 5.8; // Change this!
        public final static double MAX_LINEAR_ACCELERATION = 4.8;
        public final static double MAX_ROTATIONAL_SPEED = Math.PI * 2 * 1.5;
        public final static double MAX_ROTATIONAL_ACCELERATION = Math.PI * 2 * 1.5;
    }

    public static class Arm { // Arm Contstants
        public static final double ARM_DOWN_SENSOR_LIMIT = 1240; //TODO: fix all of these lol
        public static final double ARM_UP_SENSOR_LIMIT = 300;
        public static final double ARM_DOWN_ANGLE = 1.8;
        public static final double ARM_UP_ANGLE = 0.3;
    }

    public static class Intake { // Intake Constants
        public static final double intakeRunSpeed = -0.55; //TODO: fix
    }

    public static class Vision { // Vision Constants
        /* Camera */
        public static final int width = 300;//TODO: fix
        public static final int height = 200;

        
        
    }

    public static class Robot {//Robot Constants
        //TODO: add
    }
}