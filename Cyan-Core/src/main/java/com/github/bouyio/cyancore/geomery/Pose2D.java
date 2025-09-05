package com.github.bouyio.cyancore.geomery;

/**
 * A container for 2D coordinate and heading info.
 * Enhanced with input validation and performance optimizations.
 *
 * @author Bouyio (https://github.com/bouyio)
 * @author Gvol (https://github.com/Gvolexe)
 */
public class Pose2D {
    private final double x;
    private final double y;
    private final double theta;

    /**
     * <p>Creates a Pose object with specified coordinates and heading.<p/>
     * @param x The x coordinates of the pose.
     * @param y The y coordinates of the pose.
     * @param theta The heading of the pose.
     * */
    public Pose2D(double x, double y, double theta) {
        // Optimized: Add input validation
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(theta)) {
            throw new IllegalArgumentException("Pose values must be finite");
        }
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    /**@return The angle of the pose.*/
    public double getTheta() {
        return theta;
    }

    /**@return The coordinates of the pose in the x axis.*/
    public double getX() {
        return x;
    }

    /**@return The coordinates of the pose in the y axis.*/
    public double getY() {
        return y;
    }

    /**
     * <p>Formats the x and y coordinates of the pose as a point.<p/>
     * @return The coordinates of the pose as a point.
     * */
    public Point toPoint() {
        return new Point(x, y);
    }

    /**
     * <p>Calculates the distance between this pose and another pose.<p/>
     * @param other The other pose.
     * @return The distance between the poses.
     * */
    public double distanceTo(Pose2D other) {
        if (other == null) {
            throw new IllegalArgumentException("Other pose cannot be null");
        }
        return Math.hypot(this.x - other.x, this.y - other.y);
    }

    /**
     * <p>Calculates the angle difference between this pose and another pose.<p/>
     * @param other The other pose.
     * @return The angle difference in radians.
     * */
    public double angleDifference(Pose2D other) {
        if (other == null) {
            throw new IllegalArgumentException("Other pose cannot be null");
        }
        return Math.atan2(other.y - this.y, other.x - this.x);
    }

    /**
     * <p>Creates a string representation of this pose.<p/>
     * @return String representation of the pose.
     * */
    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "Pose2D(x=%.3f, y=%.3f, θ=%.3f)", x, y, theta);
    }
}
