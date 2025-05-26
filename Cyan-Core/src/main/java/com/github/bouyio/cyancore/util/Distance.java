package com.github.bouyio.cyancore.util;

public class Distance {

    private final double value;
    private final DistanceUnit unitOfMeasurement;

    public Distance(double value, DistanceUnit unit) {
        this.value = value;
        this.unitOfMeasurement = unit;
    }

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

    public double getInCM() {
        return value * unitOfMeasurement.toCM;
    }

    public double getInMeters() {
        return value * unitOfMeasurement.toMeters;
    }

    public double getInInches() {
        return value * unitOfMeasurement.toInches;
    }

    public double getInFeet() {
        return value * unitOfMeasurement.toFeet;
    }

    public double getAs(DistanceUnit unit) {
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

    public double getRawValue() {
        return value;
    }
}
