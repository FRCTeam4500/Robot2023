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
     * Returns a value bounded above by {@code max}, and below by {@code min}.
     *
     * @param min    the lower bound
     * @param max    the upper bound
     * @param output the value to be clamped
     * @return {@code output} if it's within the bounds, otherwise whichever bound is closer.
     */
    public static double clamp(double min, double max, double output) {
        return Math.min(max, Math.max(min, output));
    }

    /**
     * Computes the dot product of {@code a} and {@code b}.
     *
     * @see <a href="https://en.wikipedia.org/wiki/Dot_product">Wikipedia article on Dot Products</a>
     * @param a A Translation2D, understood as a vector
     * @param b Another Translation2D, understood as a vector
     * @return Their dot product (x1*x2+y1*y2)
     */
    public static double dot(Translation2d a, Translation2d b){
        return a.getX()*b.getX()+a.getY()*b.getY();
    }

    /**
     * Computes the angle between {@code a} and {@code b}.
     *
     * @param a A Translation2D, for our purposes a complex number or a vector
     * @param b Another Translation2D, for our purposes a complex number or a vector
     * @return An angle from <i>-pi</i> to <i>pi</i>.
     */
    public static double angleBetween(Translation2d a, Translation2d b){
        return Math.atan2(a.getY()*b.getX() - a.getX()*b.getY(), a.getX()*b.getX() + a.getY()*b.getY());
        // do this by understanding a and b as complex numbers, divide them (this is the same as subtracting the angles)
        // normally, complex division would require rescaling, but we only care about angle, so we can skip that.
        // then, use Java's Math library's atan2 function. It accepts arguments in the order y,x.
    }

    /**
     * Returns the length of {@code a} in the direction of {@code b}
     * @param a The vector for length
     * @param b The vector for direction
     * @return How far {@code a} goes in {@code b}'s direction.
     * @see <a href="https://en.wikipedia.org/wiki/Dot_product#Scalar_projection_and_first_properties">
     *     The Wikipedia article on dot products has a section on projections.
     *     </a>
     */
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
     * @param a The vector to normalize
     * @return A vector in the same direction as <i>a</i>, but with length 1.
     */
    public static Translation2d normalize(Translation2d a){
        return a.div(a.getNorm());
    }

    /**
     * Zeroes the value if it's within the deadzone.
     * @param value A number
     * @param deadzone The size of the deadzone
     * @return 0 if {@code value} is within {@code deadzone} of 0, otherwise {@code value}
     */
    public static double withHardDeadzone(double value, double deadzone){
        if(Math.abs(value) < deadzone){
            return 0;
        }else{
            return value;
        }
    }

    /**
     * A continuous deadzone.
     *
     * @param input The value to apply a deadzone to
     * @param slope The rate of change of the value when it's not dead.
     * @param deadzone The size of the deadzone.
     * @return 0 if {@code input} is within {@code deadzone} of 0.
     *         Otherwise, {@code deadzone} is added or subtracted to bring it closer to 0,
     *         and then the result is multiplied by {@code slope}
     */
    public static double withContinuousDeadzone(double input, double slope, double deadzone){
        if(input <= -deadzone){
            return (input + deadzone) * slope;
        }else if(-deadzone < input && input < deadzone){
            return 0;
        }else{
            return (input - deadzone) * slope;
        }
    }

    /**
     * A continuous deadzone with automatic slope, selected so that for {@code input} 1, it always returns 1.
     * @param input A number to deadzone
     * @param deadzone The deadzone size
     * @return A continuously deadzoned value.
     */
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

    /**
     * Returns the length of the difference of two vectors.
     * @param start The first vector
     * @param end The second vector
     * @return The length of their difference
     */
    public static double distance(Translation2d start, Translation2d end){
        var x = end.getX() - start.getX();
        var y = end.getY() - start.getY();
        return Math.sqrt(x*x + y*y);
    }
}

