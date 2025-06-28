package com.github.bouyio.cyancore.geomery;

import com.github.bouyio.cyancore.util.Distance;
import com.github.bouyio.cyancore.util.MathUtil;

import java.util.Locale;

/**
 * <p>
 *     This class stores and represents coordinates of a cartesian coordinate system with a certain
 *     distance unit of measurement.
 *     Those coordinates can be converted to any other of the supported distance units or to a
 *     standard point.
 * </p>
 * @see Point
 * @see Distance
 * @see Distance.DistanceUnit
 * @see SmartVector
 */
public class SmartPoint {
    private final Distance.DistanceUnit unitOfMeasurement;
    private final SmartVector coordinates;
    private final Distance distance;

    /**
     * <p>Creates a point with specified coordinates and unit of measurement.<p/>
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
     * @return The x and y coordinates formatted as {@link SmartVector}.
     * */
    public SmartVector getCoordinates() {
        return coordinates;
    }


    /**
     * <p>Formats the point's coordinates and unit of measurement in a form easier for debugging.<p/>
     * @return Formatted coordinates and unit of measurement.
     * */
    @Override
    public String toString() {
        return String.format(Locale.getDefault(),
                "X: %f, Y: %f, Distance Unit: %s",
                coordinates.getX().convertTo(unitOfMeasurement),
                coordinates.getY().convertTo(unitOfMeasurement),
                unitOfMeasurement.name());
    }

    /**
     * <p>Calculates the hypotenuse of the difference of the coordinates between this and the given point.<p/>
     * @param point The given point.
     * @return The distance from the given point.
     * */
    public double getDistanceFrom(SmartPoint point) {
        double x = point.getCoordinates().getX().convertTo(unitOfMeasurement);
        double y = point.getCoordinates().getX().convertTo(unitOfMeasurement);

        return MathUtil.hypotenuse(
                coordinates.getX().convertTo(unitOfMeasurement) - x,
                coordinates.getY().convertTo(unitOfMeasurement) - y);
    }

    /**
     * <p>Calculates the hypotenuse of the difference of the coordinates between this point and the given vector.<p/>
     * @param vector The given vector.
     * @return The distance from the given vector.
     * */
    public double getDistanceFrom(SmartVector vector) {
        double x = vector.getX().convertTo(unitOfMeasurement);
        double y = vector.getX().convertTo(unitOfMeasurement);

        return MathUtil.hypotenuse(
                coordinates.getX().convertTo(unitOfMeasurement) - x,
                coordinates.getY().convertTo(unitOfMeasurement) - y);
    }


    /**
     * <p>Calculates and returns the angle of the polar coordinates of the point in Radians.<p/>
     * */
    public double pointAngle() {
        return Math.atan2(
                coordinates.getY().convertTo(unitOfMeasurement),
                coordinates.getX().convertTo(unitOfMeasurement));
    }

    /**
     * <p>Formats the points coordinates into a point.<p/>
     * @return The formatted coordinates.
     * */
    public Point getAsPoint() {
        return new Point(
                coordinates.getX().convertTo(unitOfMeasurement),
                coordinates.getY().convertTo(unitOfMeasurement)
                );
    }

    /**
     * <p>Converts the coordinates to a given distance unit of measurement and formats the as a point.<p/>
     * @param unit The give distance unit.
     * @return The converted and formatted coordinates.
     * */
    public Point getAsPoint(Distance.DistanceUnit unit) {
        return new Point(
                coordinates.getX().convertTo(unit),
                coordinates.getY().convertTo(unit)
        );
    }
}
