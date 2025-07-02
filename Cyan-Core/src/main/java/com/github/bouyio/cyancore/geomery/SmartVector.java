package com.github.bouyio.cyancore.geomery;

import com.github.bouyio.cyancore.util.Distance;

/**
 * <p>
 *     This class stores and represents coordinates of a cartesian coordinate system with a certain
 *     distance unit of measurement.
 *     Those coordinates can be converted to any other of the supported distance units or to a
 *     standard pose 2d.
 * </p>
 * @see Pose2D
 * @see Distance
 * @see Distance.DistanceUnit
 */
public class SmartVector {
    private final Distance x;
    private final Distance y;
    private final Distance.DistanceUnit unitOfMeasurement;

    /**
     * <p>
     *     Creates a smart vector with given coordinates and distance unit.
     * </p>
     *
     * @param unit The given distance unit.
     * @param x The x of the given coordinates.
     * @param y The y of the given coordinates.
     * */
    public SmartVector(Distance.DistanceUnit unit, double x, double y) {
        this.x = new Distance(x, unit);
        this.y = new Distance(y, unit);
        unitOfMeasurement = unit;
    }

    /**@return The raw x coordinates of the smart vector.*/
    public Distance getX() {
        return x;
    }

    /**@return The raw y coordinates of the smart vector.*/
    public Distance getY() {
        return y;
    }

    /**
     * <p>
     *     Converts and returns the coordinates to a given distance unit as a {@link Pose2D}.
     * </p>
     *
     * @param unit The given distance unit.
     * @return The converted pose.
     * */
    public Pose2D getAsPose(Distance.DistanceUnit unit) {
        return new Pose2D(x.convertTo(unit), y.convertTo(unit), 0);
    }

    /**
     *
     * <p>
     *     Formats and returns the smart vector as a {@link Pose2D} with its native distance unit.
     * </p>
     *
     * @return The formatted pose.
     * */
    public Pose2D getAsPose() {
        return new Pose2D(x.convertTo(unitOfMeasurement), y.convertTo(unitOfMeasurement), 0);
    }

    /**@return The unit of measurement of the smart vector.*/
    public Distance.DistanceUnit getUnitOfMeasurement() {
        return unitOfMeasurement;
    }
}
