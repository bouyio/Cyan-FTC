package bouyio.cyancore.util;

public class MathUtil {
    public static double wrapAngle(double angle) {
        return Math.abs(angle) > Math.PI ? angle % Math.PI : angle;
    }
}
