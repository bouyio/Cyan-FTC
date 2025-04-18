package bouyio.cyancore.geomery;

import java.util.Locale;

import bouyio.cyancore.util.MathUtil;

public class Point {
    private final Pose2D coordinates;
    private final double distance;

    public double getDistanceFromOrigin() {
        return distance;
    }

    public Pose2D getCoordinates() {
        return coordinates;
    }

    public Point(double x, double y) {
        this.coordinates = new Pose2D(x, y, 0);
        this.distance = Math.sqrt(Math.pow(coordinates.getX(), 2) + Math.pow(coordinates.getY(), 2));
    }

    @Override
    public String toString() {
        return String.format(Locale.getDefault(),
                "X: %f, Y: %f, Theta: %f",
                coordinates.getX(),
                coordinates.getY(),
                coordinates.getTheta());
    }

    public double getDistanceFrom(Point point) {
        return MathUtil.hypotenuse(
                coordinates.getX() - point.coordinates.getX(),
                coordinates.getY() - point.coordinates.getY());
    }

    public double getDistanceFrom(Pose2D pose) {
        return MathUtil.hypotenuse(
                coordinates.getX() - pose.getX(),
                coordinates.getY() - pose.getY());
    }

    public double getDistanceFrom(double x, double y) {
        return MathUtil.hypotenuse(
                coordinates.getX() - x,
                coordinates.getY() - y);
    }

    public double pointAngle() {
        return Math.atan2(coordinates.getY(), coordinates.getX());
    }
}
