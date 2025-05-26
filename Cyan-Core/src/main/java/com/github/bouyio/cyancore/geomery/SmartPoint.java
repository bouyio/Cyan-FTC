package com.github.bouyio.cyancore.geomery;

import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;

import java.util.Locale;

public class SmartPoint {
    private final Distance.DistanceUnit unitOfMeasurement;
    private final SmartVector coordinates;
    private final Distance distance;

    /**
     * <p>Creates a point with specified coordinates.<p/>
     * @param x The x coordinates of the point.
     * @param y The y coordinates of the point.
     * */
    public SmartPoint(Distance.DistanceUnit unit, double x, double y) {
        this.coordinates = new SmartVector(unit, x, y);
        this.unitOfMeasurement = unit;

        this.distance = new Distance(
                Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)),
                unit
        );
    }

    /**
     * @return The distance from the origin of the two coordinate axes (0,0).
     * */
    public Distance getDistanceFromOrigin() {
        return distance;
    }

    /**
     * @return The x and y coordinates formatted as {@link Pose2D}.
     * */
    public SmartVector getCoordinates() {
        return coordinates;
    }


    /**
     * <p>Formats the point's coordinates in a form easier for debugging.<p/>
     * @return Formatted coordinates.
     * */
    @Override
    public String toString() {
        return String.format(Locale.getDefault(),
                "X: %f, Y: %f",
                coordinates.getX().getAs(unitOfMeasurement),
                coordinates.getY().getAs(unitOfMeasurement));
    }

    /**
     * <p>Calculates the hypotenuse of the difference of the coordinates between this and the given point.<p/>
     * @param point The given point.
     * @return The distance from the given point.
     * */
    public double getDistanceFrom(SmartPoint point) {
        double x = point.getCoordinates().getX().getAs(unitOfMeasurement);
        double y = point.getCoordinates().getX().getAs(unitOfMeasurement);

        return MathUtil.hypotenuse(
                coordinates.getX().getAs(unitOfMeasurement) - x,
                coordinates.getY().getAs(unitOfMeasurement) - y);
    }

    /**
     * <p>Calculates the hypotenuse of the difference of the coordinates between this point and the given pose.<p/>
     * @param vector The given pose.
     * @return The distance from the given pose.
     * */
    public double getDistanceFrom(SmartVector vector) {
        double x = vector.getX().getAs(unitOfMeasurement);
        double y = vector.getX().getAs(unitOfMeasurement);

        return MathUtil.hypotenuse(
                coordinates.getX().getAs(unitOfMeasurement) - x,
                coordinates.getY().getAs(unitOfMeasurement) - y);
    }


    /**
     * <p>Calculates and returns the angle of the polar coordinates of the point in Radians.<p/>
     * */
    public double pointAngle() {
        return Math.atan2(
                coordinates.getY().getAs(unitOfMeasurement),
                coordinates.getX().getAs(unitOfMeasurement));
    }

    public Point getAsPoint() {
        return new Point(
                coordinates.getX().getAs(unitOfMeasurement),
                coordinates.getY().getAs(unitOfMeasurement)
                );
    }

    public Point getAsPoint(Distance.DistanceUnit unit) {
        return new Point(
                coordinates.getX().getAs(unit),
                coordinates.getY().getAs(unit)
        );
    }
}
