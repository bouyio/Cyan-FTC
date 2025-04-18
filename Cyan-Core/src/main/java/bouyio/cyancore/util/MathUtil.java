package bouyio.cyancore.util;

public class MathUtil {
    public static double wrapAngle(double angle) {
        return Math.abs(angle) > Math.PI ? angle % Math.PI : angle;
    }

    public static double shiftAngle(double angle, double offset) {
        angle += offset;

        if (Math.abs(angle) > 180) {
            angle %= 180;

            angle = (180 - Math.abs(angle)) * -Math.signum(angle);
        }

        return angle;
    }

    public static double hypotenuse(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public static boolean isValueInRange(double min, double max, double val) {
        return min <= val && val <= max;
    }

    public static double filterImu(double input, double previousValue, double deltaAngle) {
        if (input - previousValue >= 180){
            deltaAngle -= 360;
        }
        else if (input - previousValue <= -180){
            deltaAngle += 360;
        }

        return deltaAngle;
    }

    public static int sgn(double in) {
        return in < 0 ? -1 : 1;
    }

    public static double clamp(double min, double max, double val) {
        return val > max ? max : Math.max(min, val);
    }
}
