package bouyio.cyancore.geomery;

import java.util.Locale;

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
        return Math.sqrt(Math.pow(coordinates.getX() - point.coordinates.getX(), 2) +
                Math.pow(coordinates.getY() - point.coordinates.getY(), 2));
    }

    public double getDistanceFrom(Pose2D pose) {
        return Math.sqrt(Math.pow(coordinates.getX() - pose.getX(), 2) +
                Math.pow(coordinates.getY() - pose.getY(), 2));
    }
}
