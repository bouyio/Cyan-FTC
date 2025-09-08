package com.github.bouyio.cyancore.geomery;

public class Vector2D {
    private final double x;
    private final double y;

    private final double r;
    private final double theta;

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

    public double getCartesianX() {
        return x;
    }

    public double getCartesianY() {
        return y;
    }

    public double getRadialR() {
        return r;
    }

    public double getRadialTheta() {
        return theta;
    }
}
