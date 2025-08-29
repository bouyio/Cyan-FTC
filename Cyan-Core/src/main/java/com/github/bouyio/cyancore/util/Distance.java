package com.github.bouyio.cyancore.util;

/**
 * <p>
 *     This class represents linear distances with their respective distance unit.
 *     This class contains methods to convert to any supported unit; centimeters, meters, inches, feet.
 * </p>
 * @see com.github.bouyio.cyancore.geomery.SmartPoint
 * @see com.github.bouyio.cyancore.geomery.SmartVector
 * */
public class Distance {

    private final double value;
    private final DistanceUnit unitOfMeasurement;

    /**Creates a Distance object with specified value and distance unit.*/
    public Distance(double value, DistanceUnit unit) {
        this.value = value;
        this.unitOfMeasurement = unit;
    }

    /**
     * <p>
     *     This class represents the supported linear distance units and contains methods to
     *     convert between them.
     * </p>
     * */
    public enum DistanceUnit {
        CM(1, 0.01, 0.0328084, 0.39370079),
        METER(100, 1, 3.2808399, 39.3700787),
        INCH(2.54, 0.0254, 0.08333333, 1),
        FOOT(30.48, 0.3048, 1, 12);

        final double toCM;
        final double toMeters;
        final double toFeet;
        final double toInches;

        DistanceUnit(double toCM, double toMeters, double toFeet, double toInches) {
            this.toCM = toCM;
            this.toMeters = toMeters;
            this.toFeet = toFeet;
            this.toInches = toInches;
        }
    }

    /**
     * <p>
     *     Converts the linear distance from its native unit of measurement to centimeters.
     * </p>
     * @return The converted unit
     * */
    public double getInCM() {
        return value * unitOfMeasurement.toCM;
    }

    /**
     * <p>
     *     Converts the linear distance from its native unit of measurement to meters.
     * </p>
     * @return The converted unit
     * */
    public double getInMeters() {
        return value * unitOfMeasurement.toMeters;
    }

    /**
     * <p>
     *     Converts the linear distance from its native unit of measurement to inches.
     * </p>
     * @return The converted unit
     * */
    public double getInInches() {
        return value * unitOfMeasurement.toInches;
    }

    /**
     * <p>
     *     Converts the linear distance from its native unit of measurement to feet.
     * </p>
     * @return The converted unit
     * */
    public double getInFeet() {
        return value * unitOfMeasurement.toFeet;
    }

    /**
     * <p>
     *     Converts the linear distance from its native unit of measurement to a given unit of
     *     measurement.
     * </p>
     * @param unit The given unit of measurement
     * @return The converted unit
     * */
    public double convertTo(DistanceUnit unit) {
        switch (unit) {
            case CM:
                return getInCM();
            case METER:
                return getInMeters();
            case FOOT:
                return getInFeet();
            case INCH:
                return getInInches();
            default:
                throw new RuntimeException("Distance unit is either undefined or null.");
        }
    }

    /**@return The raw value of the distance object.*/
    public double getRawValue() {
        return value;
    }

    /**@return The unit of measurement attached to the distance object.*/
    public DistanceUnit getUnitOfMeasurement() {
        return unitOfMeasurement;
    }
}
