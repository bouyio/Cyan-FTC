package bouyio.cyancore.geomery;

public class Point {
    private final Pose2D coordinates;
    private final double distance;

    public double getDistance() {
        return distance;
    }

    public Pose2D getCoordinates() {
        return coordinates;
    }

    public Point(double x, double y) {
        this.coordinates = new Pose2D(x, y, 0);
        this.distance = Math.sqrt(Math.pow(coordinates.getX(), 2) + Math.pow(coordinates.getY(), 2));
    }
}
