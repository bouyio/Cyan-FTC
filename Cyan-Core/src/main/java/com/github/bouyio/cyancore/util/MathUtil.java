package com.github.bouyio.cyancore.util;

/**<p>Contains useful functions for mathematical and geometrical operations.<p/>*/
public class MathUtil {

    /**
     * <p>Wraps the given angle as per full rotations; 2π.<p/>
     * @param angle The given angle in Radians.
     * @return The wrapped angle in Radians.
     * */
    public static double wrapAngle(double angle) {
        return Math.abs(angle) > 2*Math.PI ? angle % 2*Math.PI : angle;
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

        if (Math.abs(angle) > 180) {
            angle %= 180;

            angle = (180 - Math.abs(angle)) * -Math.signum(angle);
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
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
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

    public static double clamp(double min, double max, double val) {
        return val > max ? max : Math.max(min, val);
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
