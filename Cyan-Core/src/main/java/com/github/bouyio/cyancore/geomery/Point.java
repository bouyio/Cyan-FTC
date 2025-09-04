package com.github.bouyio.cyancore.geomery;

import java.util.Locale;

import com.github.bouyio.cyancore.util.MathUtil;

/**
 * Represents a point in a two-dimensional plane.
 * Optimized with numerical stability improvements using Math.hypot().
 *
 * @see Pose2D
 * @author Bouyio (https://github.com/bouyio)
 * @author Gvol (https://github.com/Gvolexe)
 */
public class Point {
    private final Pose2D coordinates;
    private final double distance;

    /**
     * <p>Creates a point with specified coordinates.<p/>
     * @param x The x coordinates of the point.
     * @param y The y coordinates of the point.
     * */
    public Point(double x, double y) {
        // Optimized: Add input validation and more efficient distance calculation
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            throw new IllegalArgumentException("Point coordinates must be finite values");
        }
        this.coordinates = new Pose2D(x, y, 0);
        // Optimized: Use Math.hypot for better numerical stability
        this.distance = Math.hypot(coordinates.getX(), coordinates.getY());
    }

    /**
     * @return The distance from the origin of the two coordinate axes (0,0).
     * */
    public double getDistanceFromOrigin() {
        return distance;
    }

    /**
     * @return The x and y coordinates formatted as {@link Pose2D}.
     * */
    public Pose2D getCoordinates() {
        return coordinates;
    }


    /**
     * <p>Formats the point's coordinates in a form easier for debugging.<p/>
     * @return Formatted coordinates.
     * */
    @Override
    public String toString() {
        return String.format(Locale.getDefault(),
                "X: %f, Y: %f, Theta: %f",
                coordinates.getX(),
                coordinates.getY(),
                coordinates.getTheta());
    }

    /**
     * <p>Calculates the hypotenuse of the difference of the coordinates between this and the given point.<p/>
     * @param point The given point.
     * @return The distance from the given point.
     * */
    public double getDistanceFrom(Point point) {
        return MathUtil.hypotenuse(
                coordinates.getX() - point.coordinates.getX(),
                coordinates.getY() - point.coordinates.getY());
    }

    /**
     * <p>Calculates the hypotenuse of the difference of the coordinates between this point and the given pose.<p/>
     * @param pose The given pose.
     * @return The distance from the given pose.
     * */
    public double getDistanceFrom(Pose2D pose) {
        return MathUtil.hypotenuse(
                coordinates.getX() - pose.getX(),
                coordinates.getY() - pose.getY());
    }

    /**
     * <p>Calculates the hypotenuse of the difference between this point's coordinates and the given coordinates.<p/>
     * @param x The x of the given coordinates.
     * @param y The y of the given coordinates.
     * @return The distance from the given coordinates.
     * */
    public double getDistanceFrom(double x, double y) {
        return MathUtil.hypotenuse(
                coordinates.getX() - x,
                coordinates.getY() - y);
    }

    /**
     * <p>Calculates and returns the angle of the polar coordinates of the point in Radians.<p/>
     * */
    public double pointAngle() {
        return Math.atan2(coordinates.getY(), coordinates.getX());
    }
}
