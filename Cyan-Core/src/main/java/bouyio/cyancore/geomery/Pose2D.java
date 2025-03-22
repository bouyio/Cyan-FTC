package bouyio.cyancore.geomery;

public class Pose2D {
    private final double x;
    private final double y;
    private final double theta;

    public Pose2D(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double getTheta() {
        return theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
