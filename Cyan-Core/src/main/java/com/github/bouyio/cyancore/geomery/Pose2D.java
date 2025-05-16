package com.github.bouyio.cyancore.geomery;

/**
 * <p>A container for 2D coordinate and heading info.<p/>
 * */
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
}
