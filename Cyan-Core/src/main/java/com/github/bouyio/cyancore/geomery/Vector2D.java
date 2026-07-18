package com.github.bouyio.cyancore.geomery;

/**
 * <p>This class represents a two-dimensional vector.</p>
 * */
public class Vector2D {
    private final double x;
    private final double y;

    private final double r;
    private final double theta;

    /**
     * <p>Creates a vector with specified x and y values.<p/>
     * @param x The x value of the point.
     * @param y The y value of the point.
     * */
    public Vector2D(double x, double y) {

        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            throw new IllegalArgumentException("Point coordinates must be finite values");
        }

        if (Double.isNaN(x) || Double.isNaN(y)) {
            throw new IllegalArgumentException("Point coordinates must be numbers");
        }

        this.x = x;
        this.y = y;

        r = Math.hypot(x, y);
        theta = Math.atan2(x, y);
    }

    /**@return The cartesian x component of the vector.*/
    public double getCartesianX() {
        return x;
    }

    /**@return The cartesian y component of the vector.*/
    public double getCartesianY() {
        return y;
    }

    /**@return The radial r component of the vector.*/
    public double getRadialR() {
        return r;
    }

    /**@return The radial theta component of the vector.*/
    public double getRadialTheta() {
        return theta;
    }
}
