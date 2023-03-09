/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * This is a simple container for math functions which are useful, but are not included in the
 * standard {@link Math} class.
 */

// Feel free to add any useful math-related functions to this class. Please do not implement any
// non-static methods.
public class ExtendedMath {
    /**
     * Clamps an output between {@code min} and {@code max}. "Clamping" refers to restricting a
     * value between a minimum and a maximum. If the given value is below the minimum, the returned
     * value is equal to the minimum. If the given value is above the maximum, the returned value is
     * equal to the maximum. If neither of these conditions are met, the given value is returned as
     * is.
     *
     * @param min    the value minimum
     * @param max    the value maximum
     * @param output the value to be clamped
     * @return the clamped value
     */
    public static double clamp(double min, double max, double output) {
        return Math.min(max, Math.max(min, output));
    }
    public static double dot(Translation2d a, Translation2d b){
        return a.getX()*b.getX()+a.getY()*b.getY();
    }
    public static double angleBetween(Translation2d a, Translation2d b){
        return Math.acos(dot(a,b)/(a.getNorm()*b.getNorm()));
    }
    public static double scalarProjectionOf(Translation2d a, Translation2d b){
        var norm = b.getNorm();
        if(norm == 0){
            return 0;
        }else{
            return dot(a,b)/norm;
        }
    }
    /**
     * Returns a normalized vector
     * @param a
     * @return
     */
    public static Translation2d normalize(Translation2d a){
        return a.div(a.getNorm());
    }
    public static double withHardDeadzone(double value, double deadzone){
        if(Math.abs(value) < deadzone){
            return 0;
        }else{
            return value;
        }
    }
    public static double withContinuousDeadzone(double input, double slope, double deadzone){
        if(input <= -deadzone){
            return (input + deadzone) * slope;
        }else if(-deadzone < input && input < deadzone){
            return 0;
        }else{
            return (input - deadzone) * slope;
        }
    }
    public static double withContinuousDeadzone(double input, double deadzone){
        return withContinuousDeadzone(input, (1/(1-deadzone)),deadzone);
    }
    /**
     * A custom mod function which returns a remainder with the same sign as the dividend. This is
     * different from using {@code %}, which returns the remainder with the same sign as the
     * divisor.
     *
     * @param a the dividend
     * @param n the divisor
     * @return the remainder with the same sign as {@code a}
     */
    public static double customMod(double a, double n) {
        return a - Math.floor(a / n) * n;
    }

    /**
     * Calculates the shortest radian to a given angle, assuming that all angles that are 2 pi away
     * from each other are equivalent.
     *
     * @param currentAngle the starting angle
     * @param targetAngle  the final angle
     * @return the smallest difference and direction between these two angles
     */
    public static double getShortestRadianToTarget(double currentAngle, double targetAngle) {
        double actualDifference = targetAngle - currentAngle;
        double shortestDifference = customMod(actualDifference + Math.PI, 2 * Math.PI) - Math.PI;
        return shortestDifference;
    }
    public static double distance(Translation2d start, Translation2d end){
        var x = end.getX() - start.getX();
        var y = end.getY() - start.getY();
        return Math.sqrt(x*x + y*y);
    }

    /**
     * Uses bigsad Euler Angles math to get the overall angle of the robot.
     * @param yaw the yaw angle in radians - Rotation about the Z axis
     * @param pitch the pitch angle in radians - Rotation about the X axis
     * @param roll the roll angle in radians - Rotation about the Y axis
     * @return the overall angle of the robot in radians
     */
    public static double getOverallAngle(double yaw, double pitch, double roll) {

        // Set the ground plane normal vector
        double[] n1 = {0.0, 0.0, 1.0};

        // Calculate the rotation matrices
        double[][] Rx = {{1.0, 0.0, 0.0},
                {0.0, Math.cos(pitch), -Math.sin(pitch)},
                {0.0, Math.sin(pitch), Math.cos(pitch)}};

        double[][] Ry = {{Math.cos(roll), 0.0, Math.sin(roll)},
                {0.0, 1.0, 0.0},
                {-Math.sin(roll), 0.0, Math.cos(roll)}};

        double[][] Rz = {{Math.cos(yaw), -Math.sin(yaw), 0.0},
                {Math.sin(yaw), Math.cos(yaw), 0.0},
                {0.0, 0.0, 1.0}};

        // Calculate the normal vector of the rotated plane
        double[] n2 = new double[3];
        for (int i = 0; i < 3; i++) {
            n2[i] = 0.0;
            for (int j = 0; j < 3; j++) {
                n2[i] += Rz[i][j] * Ry[j][i] * Rx[j][i] * n1[j];
            }
        }

        // Calculate the dot product of the two normal vectors
        double dotProduct = 0.0;
        for (int i = 0; i < 3; i++) {
            dotProduct += n1[i] * n2[i];
        }

        // Calculate the overall tilt angle in degrees
        return Math.acos(dotProduct);

    }
}

