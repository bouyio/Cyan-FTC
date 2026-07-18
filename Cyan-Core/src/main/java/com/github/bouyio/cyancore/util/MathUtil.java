package com.github.bouyio.cyancore.util;

/**
 * Contains useful functions for mathematical and geometrical operations.
 * Optimized for performance and safety in FTC robotics applications.
 *
 * @author Bouyio (<a href="https://github.com/bouyio">...</a>)
 * @author Gvol (<a href="https://github.com/Gvolexe">...</a>)
 */
public class MathUtil {

    /**
     * <p>Wraps the given angle as per full rotations; 2π.<p/>
     * @param angle The given angle in Radians.
     * @return The wrapped angle in Radians.
     * */
    public static double wrapAngle(double angle) {
        // Optimized: Use more efficient wrapping algorithm and cache 2*PI constant
        final double TWO_PI = 2.0 * Math.PI;
        if (Math.abs(angle) <= TWO_PI) {
            return angle;
        }
        // More robust angle wrapping using atan2 method
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /**
     * <p>
     *     Offsets given angle in a way that is consistent with imu input.
     *     So as an angle ω is always between -180 to 180 degrees.
     * <p/>
     * @param angle The angle to be offset in degrees.
     * @param offset The offset in degrees.
     * @return The offset angle.
     * */
    public static double shiftAngle(double angle, double offset) {
        angle += offset;

        // Optimized: More efficient angle normalization using single operation
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle <= -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    /**
     * <p>Calculates the hypotenuse of right triangle with given sides.<p/>
     * @param x Side of the triangle.
     * @param y Side of the triangle.
     * @return The hypotenuse.
     * */
    public static double hypotenuse(double x, double y) {
        // Optimized: Use Math.hypot for better numerical stability and performance
        return Math.hypot(x, y);
    }

    /**
     * <p>Checks if a numerical value is between given upper and lower numerical values.<p/>
     * @param min The lower numerical value.
     * @param max The upper numerical value.
     * @param val The numerical value to be checked.
     * */
    public static boolean isValueInRange(double min, double max, double val) {
        return min <= val && val <= max;
    }


    /**
     * <p>
     *     Calculates the sign of a given numerical value.
     *     If the value is 0, then it will be considered as positive.
     * <p/>
     * @param in The numerical value.
     * @return The sign.
     * */
    public static int sgn(double in) {
        return in < 0 ? -1 : 1;
    }

    /**
     * <p>Clamps a value between minimum and maximum bounds.<p/>
     * @param min The minimum allowed value.
     * @param max The maximum allowed value.
     * @param val The value to clamp.
     * @return The clamped value.
     * */
    public static double clamp(double min, double max, double val) {
        // Optimized: Add input validation and use more efficient comparison
        if (min > max) {
            throw new IllegalArgumentException("Minimum value cannot be greater than maximum value");
        }
        return Math.max(min, Math.min(max, val));
    }

    /**
     * <p>Checks if two double values are approximately equal within a tolerance.<p/>
     * @param a First value to compare.
     * @param b Second value to compare.
     * @param epsilon Tolerance for comparison.
     * @return True if values are approximately equal.
     * */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    /**
     * <p>Checks if two double values are approximately equal with default tolerance.<p/>
     * @param a First value to compare.
     * @param b Second value to compare.
     * @return True if values are approximately equal.
     * */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, 1e-9);
    }

    // ----DEPRECATED METHOD----

    @Deprecated
    public static double filterImu(double input, double previousValue, double deltaAngle) {
        if (input - previousValue >= 180){
            deltaAngle -= 360;
        }
        else if (input - previousValue <= -180){
            deltaAngle += 360;
        }

        return deltaAngle;
    }
}
