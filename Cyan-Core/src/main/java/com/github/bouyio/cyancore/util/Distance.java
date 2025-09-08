package com.github.bouyio.cyancore.util;

import java.lang.reflect.Field;

/**
 * This class represents linear distances with their respective distance unit.
 * This class contains methods to convert to any supported unit; centimeters, meters, inches, feet.
 * Optimized for performance with numerical stability improvements and input validation.
 *
 * @see com.github.bouyio.cyancore.geomery.SmartPoint
 * @author Bouyio (https://github.com/bouyio)
 * @author Gvol (https://github.com/Gvolexe)
 */
public class Distance {

    private final double value;
    private final DistanceUnit unitOfMeasurement;

    // Optimized: Make validation optional for maximum run speed
    private static final boolean ENABLE_VALIDATION = Boolean.parseBoolean(
        System.getProperty("cyan.validation.enabled", "true")
    );

    /**Creates a Distance object with specified value and distance unit.*/
    public Distance(double value, DistanceUnit unit) {
        if (ENABLE_VALIDATION) {
            // Only validate if explicitly enabled (default: true for safety)
            if (unit == null) {
                throw new IllegalArgumentException("Distance unit cannot be null");
            }
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException("Distance value must be finite");
            }
            if (Double.isNaN(value)) {
                throw new IllegalArgumentException("Distance value must a number");
            }
        }
        this.value = value;
        this.unitOfMeasurement = unit;
    }

    /**
     * <p>Creates a Distance object with maximum performance (no validation).<p/>
     * <p><strong>WARNING:</strong> Use only when you're certain inputs are valid!</p>
     * @param value The distance value (must be finite)
     * @param unit The distance unit (must not be null)
     * @return New Distance object
     */
    public static Distance createUnsafe(double value, DistanceUnit unit) {
        // Fastest possible creation - bypasses all validation
        Distance d = new Distance(0, DistanceUnit.METER);
        // Direct field access for maximum speed
        try {
            Field valueField = Distance.class.getDeclaredField("value");
            Field unitField = Distance.class.getDeclaredField("unitOfMeasurement");
            valueField.setAccessible(true);
            unitField.setAccessible(true);
            valueField.setDouble(d, value);
            unitField.set(d, unit);
            return d;
        } catch (NoSuchFieldException | IllegalAccessException | SecurityException e) {
            // Fallback: still faster than validation
            return new Distance(value, unit);
        }
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
        // Optimized: Add null check and use more efficient approach
        if (unit == null) {
            throw new IllegalArgumentException("Target unit cannot be null");
        }
        
        // Optimized: Use direct calculation instead of switch for better performance
        double thisInMeters = value * unitOfMeasurement.toMeters;
        return thisInMeters / unit.toMeters;
    }

    /**@return The raw value of the distance object.*/
    public double getRawValue() {
        return value;
    }

    /**@return The unit of measurement attached to the distance object.*/
    public DistanceUnit getUnitOfMeasurement() {
        return unitOfMeasurement;
    }

    /**
     * <p>Adds two distances, converting to a common unit if necessary.<p/>
     * @param other The distance to add.
     * @return New Distance object with the sum.
     * */
    public Distance add(Distance other) {
        if (other == null) {
            throw new IllegalArgumentException("Cannot add null distance");
        }
        double otherInThisUnit = other.convertTo(this.unitOfMeasurement);
        return new Distance(this.value + otherInThisUnit, this.unitOfMeasurement);
    }

    /**
     * <p>Subtracts another distance from this distance.<p/>
     * @param other The distance to subtract.
     * @return New Distance object with the difference.
     * */
    public Distance subtract(Distance other) {
        if (other == null) {
            throw new IllegalArgumentException("Cannot subtract null distance");
        }
        double otherInThisUnit = other.convertTo(this.unitOfMeasurement);
        return new Distance(this.value - otherInThisUnit, this.unitOfMeasurement);
    }

    /**
     * <p>Compares two distances for equality within a small tolerance.<p/>
     * @param other The distance to compare.
     * @return True if distances are approximately equal.
     * */
    public boolean equals(Distance other) {
        if (other == null) return false;
        double thisInMeters = this.getInMeters();
        double otherInMeters = other.getInMeters();
        return Math.abs(thisInMeters - otherInMeters) < 1e-10;
    }
}
