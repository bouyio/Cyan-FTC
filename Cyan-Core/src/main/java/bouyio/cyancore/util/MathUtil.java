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
}
